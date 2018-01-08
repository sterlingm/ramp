#!/usr/bin/env python

'''
ddpg_rl_si.py corresponds to document RAMP-RL(Small Input) in Google Doc. shown below (si means small input):
https://docs.google.com/document/d/1LdOP2iUB44w9ZC3y558ZBQdhkjSF3kag1a4c_Ci1tsI/edit?usp=sharing

In this file, we construct networks and RL agent, take actions
on the environment (through environment interface), and
do learning (implemented customly).
This file itself is not a agent. The agent is implemented in keras-rl/rl/agents/ddpg.py.
This file is similar to the files in keras-rl/rl/examples/,
which means using ddpg agent (ddpg.py) in RAMP (env).
'''

## -------------------- import --------------------
import os
import sys
import numpy as np
import datetime
import gym
from gym.spaces import prng
import rospy
import time
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64
import math
import matplotlib.pyplot as plt

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

## use the keras-rl in this repository
ramp_root = os.path.join(os.path.dirname(__file__), '../../')
sys.path.append(ramp_root) # directory_name
# keras_rl_dir = os.path.join(ramp_root, 'keras-rl/')
# sys.path.append(keras_rl_dir) # directory_name
# rl_dir = os.path.join(keras_rl_dir, 'rl/')
# sys.path.append(rl_dir) # directory_name
# agents_dir = os.path.join(rl_dir, 'agents/')
# sys.path.append(agents_dir) # directory_name

from keras_rl.rl.memory import SequentialMemory
from keras_rl.rl.random import OrnsteinUhlenbeckProcess
from keras_rl.rl.agents.ddpg import DDPGAgent

## make directory of logging
home_dir = os.getenv("HOME")
cur_date = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
file_dir = home_dir + '/data/ramp/ramp_rlpara/ddpg_ramp_si/' + cur_date + '/raw_data/'
os.system('mkdir -p ' + file_dir)
file_h = open(file_dir + "ddpg_ramp_si.txt", "a")

## get directory
rlpara_root = os.path.join(os.path.dirname(__file__), '../')
lib_dir = os.path.join(rlpara_root, 'lib/')
envs_dir = os.path.join(rlpara_root, 'ramp_gym/ramp_env_interfaces/') # directory_name
sys.path.append(envs_dir)
sys.path.append(lib_dir)

## from .py_file_name import class_name
from ramp_env_interface_si import RampEnv
from f_utility import Utility

## -------------------- init --------------------
np.set_printoptions(threshold=np.inf)
ENV_NAME = 'ramp-env-interface-si'
rospy.init_node('ddpg_ramp_si', anonymous=True)
gym.undo_logger_setup()

## Get the environment and extract the dimension of action_space.
env = RampEnv()
seed_str = input("Enter a seed for random generator (must be integer): ")
seed_int = int(seed_str)
np.random.seed(seed_int) # numpy
env.seed(seed_int) # env
prng.seed(seed_int) # space
assert len(env.action_space.shape) == 1 # assert action is a vector (not a matrix or tensor)
action_size = env.action_space.shape[0] # number of scalars in one action
assert len(env.observation_space.shape) == 1 # assert observatoin is a vector (not a matrix or tensor)
observation_size = env.observation_space.shape[0] # number of scalars in one observation
assert action_size == 3
assert observation_size == 10

## publish sth. to display in rqt
si_step_pub = rospy.Publisher('ramp_collection_si_step', Int64, queue_size = 1)
si_act_normed_pub = rospy.Publisher('ramp_collection_si_act_normed', Float64MultiArray, queue_size = 1)
si_act_pub = rospy.Publisher('ramp_collection_si_act', Float64MultiArray, queue_size = 1)

## warmup rospy timer
warmup_cnt = 0
def warmupTimerCb(event):
    global warmup_cnt
    warmup_cnt += 1

warmup_duration = rospy.Rate(0.5) # warmup for 2.0s
warmup_timer = rospy.Timer(rospy.Duration(0.5), warmupTimerCb) # 0.5s per callback
is_timer_warmup = False
print("Rospy timer is warming up......")
try:
    warmup_duration.sleep()
except rospy.exceptions.ROSInterruptException as exc:
    is_timer_warmup = False
    if rospy.core.is_shutdown():
        print("Warming up of rospy timer is interrupted by customer!")
    else:
        print("Warming up of rospy timer is interrupted: " + str(exc))
    print("Warming up of rospy timer failed!")
else:
    if warmup_cnt >= 2:
        is_timer_warmup = True
        print("Warming up of rospy timer successed!")
    else:
        is_timer_warmup = False
        print("Warming up of rospy timer failed because of some unknown reasons!")
warmup_timer.shutdown()
if not is_timer_warmup:
    print("Program exited because rospy timer did not warmup!")
    sys.exit(0) # normally exit

## test interaction with environment:
'''
s = env.reset()
print("init state: " + str(s))

act = np.array([0.001, 1.0, -1.0])
print("action: " + str(act))
s, r, d, info = env.step(act)
print("s: " + str(s) + ", r: " + str(r) + ", info: " + str(info))

act = np.array([-0.002, 2.0, -1.5])
print("action: " + str(act))
s, r, d, info = env.step(act)
print("s: " + str(s) + ", r: " + str(r) + ", info: " + str(info))
'''

## model for actor (policy, a = u(s)).
#  input: normalized single motion state (10-dimensional continuous)
#  output: normalized coefficients (3-dimensional continuous)
#  TODO: number of layers, units number of each layer, activation function
actor = Sequential()
actor.add(Flatten(input_shape=(1,) + env.observation_space.shape)) # input layer, the values in the
                                                                   # observation should be normalized
                                                                   # before being fed to the actor
actor.add(Dense(20)) # hidden layer 1
actor.add(Activation('relu')) # activation function of hidden layer 1
actor.add(Dense(20)) # hidden layer 2
actor.add(Activation('relu')) # activation function of hidden layer 2
actor.add(Dense(20)) # hidden layer 3
actor.add(Activation('relu')) # activation function of hidden layer 3
actor.add(Dense(action_size)) # outpue layer
actor.add(Activation('sigmoid')) # activation function of output layer,
                                 # the output is normalized output,
                                 # need to be transfered into actual coefficients
                                 # 0 represents minimal and 1 represents maximal
print(actor.summary())

## model for critic (Q(s,a)).
#  input: normalized s and a, size is 10 + 3 = 13
#  output: just one scalar, Q value
#  TODO: number of layers, units number of each layer, activation function
action_input = Input(shape=(action_size,), name='action_input')
observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
flattened_observation = Flatten()(observation_input)
x = Concatenate()([action_input, flattened_observation]) # input layer
x = Dense(30)(x) # dense 1
x = Activation('relu')(x) # activation 1
x = Dense(30)(x) # dense 2
x = Activation('relu')(x) # activation 2
x = Dense(30)(x) # dense 3
x = Activation('relu')(x) # activation 3
x = Dense(1)(x) # output layer
x = Activation('softplus')(x) # activation of output
critic = Model(inputs=[action_input, observation_input], outputs=x)
print(critic.summary())

## two target networks are cloned in ddpg.py
pass

## its initialization includes getting parameters
utility = Utility()

## experience replay buffer
#  there is smaller window with length equaling to window_length in the buffer
#  here just use window_length = 1 for simplity of understanding
replay_buffer = SequentialMemory(limit = utility.replay_buffer_size, window_length = 1)

## max number of switching the best trajectory
max_num_switch = math.ceil(utility.max_exe_time / utility.switch_period)

## the initial, not normalized, meter and second
s_init = np.array([0.0, utility.the_initial[0], utility.the_initial[1], utility.the_initial[2],
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0])

## normalize the initial
s_init_normed = utility.normalizeMotionState(s_init)

## init random action probability
#  the output noise is normalized
random_process = OrnsteinUhlenbeckProcess(size = action_size, sigma_min = 0.0, theta = utility.orn_paras['theta'],
                                          n_steps_annealing = int(utility.max_nb_exe * utility.orn_paras['percent']))
## test noise added on action:
'''
A = np.array([])
D = np.array([])
Qk = np.array([])
for i in range(utility.max_nb_exe):
    sample = random_process.sample()
    A = np.append(A, sample[0])
    D = np.append(D, sample[1])
    Qk = np.append(Qk, sample[2])
plt.plot(A)
plt.ylabel("A")
plt.show()
plt.plot(D)
plt.ylabel("D")
plt.show()
plt.plot(Qk)
plt.ylabel("Qk")
plt.show()
'''

## build the agent, in our case memory_interval must be set to 1
#  after this building, use agent.actor, agent.memory, agent.random_process if needed,
#  don't use actor, replay_buffer and random_process (see arguments of function in Python)
agent = DDPGAgent(nb_actions = action_size, actor = actor, critic = critic, critic_action_input = action_input,
                  memory = replay_buffer, nb_steps_warmup_critic = utility.nb_steps_warmup_critic,
                  nb_steps_warmup_actor = utility.nb_steps_warmup_actor, random_process = random_process,
                  gamma = utility.discount_factor, target_model_update = utility.target_net_update_factor,
                  batch_size = utility.mini_batch_size)

## conpile the agent with optimizer and metrics (it seems that the metrics is only used in logging, not in training)
#  clipnorm is the max norm of gradients, to avoid too big gradients. Note that the operation is clip, not normalization,
#  that means if the norm of gradients is already smaller than clipnorm, then there is nothing happening to the gradients.
#  lr is leaning rate of critic Q-net.
agent.compile(Adam(lr = utility.critic_lr, clipnorm = 1.0), metrics=['mae'])

## maintain some variables that are maintained in fit()
agent.training = True

## -------------------- do one learning in one execution --------------------
chg_file_inter = 100
for k in range(utility.max_nb_exe):
    file_h.write("######################################### STEP " + str(agent.step) +
                 " #########################################\n")
    print("######################################### STEP " + str(agent.step) +
          " #########################################")

    ## save the weights
    if agent.step % chg_file_inter == 0:
        weights_file_id = int(agent.step / chg_file_inter)
        weights_dir = file_dir + str(weights_file_id) + "/"
        os.system('mkdir -p ' + weights_dir)
    agent.save_weights(weights_dir + 'ddpg_{}_weights.h5f'.format(ENV_NAME), overwrite = True)
    
    action_normed = agent.forward(s_init_normed) # the noise is added in the "forward" function
    action = utility.antiNormalizeCoes(action_normed)

    ## apply new coefficients and start a new execution
    observations, reward, done, info = env.step(action)

    ## ctrl+c interrupt
    if rospy.core.is_shutdown():
        break

    ## logging and displaying
    #  need rosbag close slowly
    si_step_pub.publish(Int64(agent.step))
    si_act_normed_pub.publish(Float64MultiArray(data = action_normed.tolist()))
    si_act_pub.publish(Float64MultiArray(data = action.tolist()))
    
    file_h.write("< action >\n")
    file_h.write(str(action) + "\n")
    print("< action >")
    print(action)

    file_h.write("< reward >\n")
    file_h.write(str(reward) + "\n")
    print("< reward >")
    print(reward)

    file_h.write("< done >\n")
    file_h.write(str(done) + "\n")
    print("< done >")
    print(done)

    ## Store all observations (normalized) in the new execution into agent.memory
    #  TODO: Stora what observations (experiences) (motion states not actually being experienced may be wrong observations,
    #        because what we really evaluate is the actual execution time, only the actual experienced motion states can
    #        associated with the actual execution time)
    nb_observations = 0
    trajs = observations.best_trajectory_vector
    for traj in trajs:
        nb_observations += len(traj.trajectory.points)

    ob_id = 0
    for traj in trajs: # for each best traj.
        for motion_state in traj.trajectory.points: # for each motion state
            s = np.array([motion_state.time_from_start.to_sec()]) # time stamp
            s = np.append(s, motion_state.positions) # position (x, y, theta)
            s = np.append(s, motion_state.velocities) # velocity
            s = np.append(s, motion_state.accelerations) # acceleration
            s_normed = utility.normalizeMotionState(s) # normalization
            agent.recent_observation = s_normed
            agent.recent_action = action_normed

            if ob_id == nb_observations - 2: # second last ob., whose next ob. is the last ob.
                agent.memory.append(agent.recent_observation, agent.recent_action,
                                    reward, done, agent.training) # s0, a0, r0, terminal1, _ (r0 is determined by s1)
            elif ob_id < nb_observations - 2:
                agent.memory.append(agent.recent_observation, agent.recent_action,
                                    0.0, False, agent.training)
            else: # the last ob. is stored in backward()
                assert ob_id == nb_observations - 1

            ob_id += 1

    assert ob_id == nb_observations

    ## Updating all neural networks
    agent.backward(reward = 0.0, terminal = False)

    ## maintain some variables that are maintained in fit()
    agent.step += 1

## close file
file_h.close()

## -------------------- After training is done, we save the final weights --------------------
agent.save_weights(file_dir + 'ddpg_{}_weights.h5f'.format(ENV_NAME), overwrite = True)

## TODO: test
agent.training = False