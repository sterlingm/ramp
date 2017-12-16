#!/usr/bin/env python

'''
In this file, we construct networks and RL agent, take actions
on the environment (through environment interface), and
do learning (fit() of keras-rl or implemented customly).
This file itself is not a agent. The agent is implemented in keras-rl/rl/agents/ddpg.py.
This file is similar to the files in keras-rl/rl/examples/,
which means using ddpg agent (ddpg.py) in RAMP.
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

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess

rlpara_root = os.path.join(os.path.dirname(__file__), '../')
envs_dir = os.path.join(rlpara_root, 'ramp_gym/ramp_env_interfaces/') # directory_name
sys.path.append(envs_dir)

# from .py_file_name import class_name
from ramp_env_interface import RampEnv

## -------------------- init --------------------
rospy.init_node('ddpg_ramp', anonymous=True)
gym.undo_logger_setup()

## Get the environment and extract the dimension of action_space.
env = RampEnv()
seed_str = input("Enter a seed for random generator (must be integer): ")
seed_int = int(seed_str)
np.random.seed(seed_int) # numpy
env.seed(seed_int) # env
prng.seed(seed_int) # space
assert len(env.action_space.shape) == 1 # assert action is a vector (not a matrix)
action_dimensioin = env.action_space.shape[0] # dimension of action_space

## test code:
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
## input: state (3-dimensional continuous), output: action (3-dimensional continuous)
## output limit the amplitude of action
## TODO: number of layers, units number of each layer, activation function
actor = Sequential()
actor.add(Flatten(input_shape=(1,) + env.observation_space.shape)) # input layer, the values in the
                                                                   # observation should be normalized
                                                                   # before being fed to the actor
actor.add(Dense(10)) # hidden layer 1
actor.add(Activation('relu')) # activation function of hidden layer 1
actor.add(Dense(10)) # hidden layer 2
actor.add(Activation('relu')) # activation function of hidden layer 2
actor.add(Dense(10)) # hidden layer 3
actor.add(Activation('relu')) # activation function of hidden layer 3
actor.add(Dense(action_dimensioin)) # outpue layer
actor.add(Activation('sigmoid')) # activation function of output layer,
                                 # the output is normalized output,
                                 # need to be transfered into actual coefficients
                                 # 0 represents minimal and 1 represents maximal
print(actor.summary())

## model for critic (Q(s,a)).
## 
'''action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=(1,) + env.observation_space.shape, name='observation_input')
flattened_observation = Flatten()(observation_input)
x = Concatenate()([action_input, flattened_observation])
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(32)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)
print(critic.summary())'''