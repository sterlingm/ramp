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
import gym
from gym.spaces import prng
import rospy
import time
from std_msgs.msg import Header
import math
import matplotlib.pyplot as plt

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess

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
actor.add(Dense(observation_size)) # hidden layer 1
actor.add(Activation('relu')) # activation function of hidden layer 1
actor.add(Dense(observation_size)) # hidden layer 2
actor.add(Activation('relu')) # activation function of hidden layer 2
actor.add(Dense(observation_size)) # hidden layer 3
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
x = Dense(observation_size + action_size)(x) # dense 1
x = Activation('relu')(x) # activation 1
x = Dense(observation_size + action_size)(x) # dense 2
x = Activation('relu')(x) # activation 2
x = Dense(observation_size + action_size)(x) # dense 3
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

## build the agent
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
agent.compile(Adam(lr = utility.lr_q_net, clipnorm = 1.0), metrics=['mae'])

## -------------------- do one learning in one execution --------------------
for k in range(utility.max_nb_exe):
    action_normed = agent.forward(s_init_normed) # the noise is added in the "forward" function
    action = utility.antiNormalizeCoes(action_normed)

    observations, reward, done, info = env.step(action)