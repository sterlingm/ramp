#!/usr/bin/env python

'''
In this file, we construct a random agent that just take random
actions on the environment (through environment interface).
The main purpose of this file is to take actions on the environment
and log corresponding data (log is done by rosbag record).
This file itself includes a random agent.
This file is similar to the files in keras-rl/rl/examples/,
which means using random agent in RAMP.
'''

## -------------------- import --------------------
import os
import sys
import numpy as np
import gym
from gym.spaces import prng
import rospy
import time
from ramp_msgs.msg import RampTransition
from ramp_msgs.msg import RampObservation
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout

rlpara_root = os.path.join(os.path.dirname(__file__), '../')
envs_dir = os.path.join(rlpara_root, 'ramp_gym/ramp_env_interfaces/') # directory_name
sys.path.append(envs_dir)

# from .py_file_name import class_name
from ramp_env_interface import RampEnv

## -------------------- init --------------------
rospy.init_node('random_ramp', anonymous=True)
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


# ramp_transition_pub = rospy.Publisher('ramp_collection_ramp_transition', RampTransition, queue_size = 1)
# i_episode = 0
# while True: # unlimited episodes
#     rAll = 0.0
#     i_episode += 1
#     s = env.reset()
#     for i_step in range(120): # 120 steps per episode
#         hdr = Header(stamp = rospy.Time.now())
#         RampObservation(header = hdr, coefficient_tensor = )
#         act = env.action_space.sample() # select random action
#         s1, r, done, info = env.step(act)
#         hdr = Header(stamp = rospy.Time.now()) # be close to 's1, r, done, info = env.step(act)'
#         ## TODO: s1 (np.array) -> coe_tensor (Float64MultiArray) should be implemented as a function.
#         #        Note that the dim of coe_tensor is actually the shape of s1
#         coe_tensor = Float64MultiArray(MultiArrayLayout(dim = ), data = s1.tolist())
#         RampObservation(header = hdr, execution_time = info['calculated_execution_time'], coefficient_tensor = coe_tensor)

        
#         rAll += r
#         s = s1