#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import gym
from gym.spaces import prng
import datetime

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

ramp_root = os.path.join(os.path.dirname(__file__), '../../')
sys.path.append(ramp_root) # directory_name

from keras_rl.rl.agents.dqn_si import DQNAgentSi
from keras_rl.rl.policy import BoltzmannQPolicy
from keras_rl.rl.policy import GreedyQPolicy
from keras_rl.rl.policy import EpsGreedyQPolicy
from keras_rl.rl.memory import SequentialMemory

from ramp_rlpara.ramp_gym.ramp_env_interfaces.ramp_env_interface_sip import RampEnvSip
rospy.init_node('dqn_ramp_sip', anonymous=True)

## make directory of logging
home_dir = os.getenv("HOME")
cur_date = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
file_dir = home_dir + '/data/ramp/ramp_rlpara/dqn_ramp_sip/' + cur_date + '/raw_data/'
os.system('mkdir -p ' + file_dir)

from f_logger import RampRlLogger
## Initialize logger
coarse_logger = RampRlLogger(file_dir + "dqn_coarse_sip.csv",
                             ['exe#', 'A', 'D',
                              'exe_reward', 'exe_time', 'obs_dis',
                              'coarse_loss', 'coarse_mae', 'coarse_mean_q'])



# Seed
seed_str = input("Enter a seed for random generator (must be a integer): ")
seed_int = int(seed_str)
np.random.seed(seed_int) # numpy
prng.seed(seed_int) # space



# Get the environment and extract the number of actions.
env = RampEnvSip('ramp_sip')
nb_actions = env.action_space.n


# Test
# ob, coes = env.reset()
# print(ob)
# print(coes)

# while not rospy.core.is_shutdown():
#     ob, r, d, info = env.step(4)
#     print(ob)
#     print(r)
#     print(d)
#     print(info)



# Next, we build a very simple model Q(s,a).
model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape)) # s is (x, y, coe)
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(nb_actions)) # Q values, number is nb_actions
model.add(Activation('linear'))
print(model.summary())



init_boltz_tau = 0.3???



# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
# even the metrics!
memory = SequentialMemory(limit=100, window_length=1)
policy = BoltzmannQPolicy(tau=init_boltz_tau)
dqn = DQNAgentSi(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=300,
               target_model_update=0.001, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])



# Load weights if needed. Put this after compiling may be better.
dqn.load_weights_sip("/home/kai/data/ramp/ramp_rlpara/dqn_ramp_sip/2018-01-20_23:40:10???/raw_data/" +
                     "dqn_{}_weights.h5f".format(env.name))



# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.
log_interval = 1000
nb_max_episode_steps = 500
dqn.fitSip(env, nb_steps=5000000, log_interval=log_interval,
           nb_max_episode_steps=nb_max_episode_steps, verbose=2,
           file_dir=file_dir, logger=coarse_logger)



# # After training is done, we save the final weights.
dqn.save_weights_sip(file_dir + 'dqn_{}_weights.h5f'.format(env.name), overwrite = True)
coarse_logger.close()



# # Finally, evaluate our algorithm for 5 episodes.
# dqn.testSip(env, nb_episodes=11, visualize=False, nb_max_episode_steps=3000)