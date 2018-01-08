'''
This is not the environment itself but the interface of environment.
Observation is coefficient vector
'''

import os
import sys
import time
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import rospy
import warnings
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from ramp_msgs.msg import RampTrajectory
from ramp_msgs.msg import RampObservationOneRunning

## get directory
rlpara_root = os.path.join(os.path.dirname(__file__), '../../')
lib_dir = os.path.join(rlpara_root, 'lib/')
sys.path.append(lib_dir)

## from .py_file_name import class_name
from f_utility import Utility

class RampEnv(gym.Env):



	def rewardCallback(self, data):
		self.reward = data.data



	def __init__(self, name):
		self.name = name
		self._seed()
		
		## get various parameters
		self.utility = Utility()

		self.action_space = spaces.Box(np.array([self.utility.coe_dA_range[0],
												 self.utility.coe_dD_range[0]]),
		                               np.array([self.utility.coe_dA_range[1],
												 self.utility.coe_dD_range[1]]))

		self.observation_space = spaces.Box(np.array([self.utility.coe_A_range[0],
													  self.utility.coe_D_range[0]]),
		                                    np.array([self.utility.coe_A_range[1],
													  self.utility.coe_D_range[1]]))
		self.observation = None
		self.reward = None

		self.reward_sub = rospy.Subscriber("ramp_collection_reward_offline", Float64, self.rewardCallback)
		self.my_A_inf = 0.5
		self.my_D_inf = 0.5



	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]
		


	def _reset(self):
		## set coefficients randomly
		self.observation = self.observation_space.sample()
		# self.observation = np.array([0.0, 0.0])

		A_T = self.my_A_inf * self.observation[0].item()
		D_T = self.my_D_inf * self.observation[1].item()

		T = 1.0 / (1.0 + A_T + D_T)
		A = A_T * T
		D = D_T * T

		rospy.set_param("/ramp/eval_weight_T", T)
		rospy.set_param("/ramp/eval_weight_A", A)
		rospy.set_param("/ramp/eval_weight_D", D)
		
		return self.observation



	def _step(self, action):
		assert self.action_space.contains(action)

		self.observation += action
		self.observation = np.clip(self.observation,
		                           self.observation_space.low,
							       self.observation_space.high)

		A_T = self.my_A_inf * self.observation[0].item()
		D_T = self.my_D_inf * self.observation[1].item()

		T = 1.0 / (1.0 + A_T + D_T)
		A = A_T * T
		D = D_T * T

		rospy.set_param("/ramp/eval_weight_T", T)
		rospy.set_param("/ramp/eval_weight_A", A)
		rospy.set_param("/ramp/eval_weight_D", D)
		
		## wait the actual environment to get ready......
		print("Wait the actual environment to get ready......")
		try:
			rospy.wait_for_service("env_ready_srv")
		except rospy.exceptions.ROSInterruptException:
			print("\nCtrl+C is pressed!")
			return self.observation, 0.0, False, {}

		print("Set start_planner to true for to start one execution!")
		rospy.set_param("/ramp/start_planner", True)

		## here you can publish sth. to "/ramp_collection_.*"
		pass
		
		## wait for this execution completes......
		has_waited_exe_for = 0 # seconds
		print("Wait for this execution completes......")
		start_waiting_time = rospy.get_rostime()
		while not rospy.core.is_shutdown() and self.reward is None:
			cur_time = rospy.get_rostime()
			has_waited_exe_for = cur_time.to_sec() - start_waiting_time.to_sec() # seconds
			if has_waited_exe_for >= self.utility.max_exe_time + 20.0: # overtime
				print("Ramp_planner has been respawned from unexpected interruption, will set start_planner to true again.")
				print("Wait the actual environment to get ready......")

				try:
					rospy.wait_for_service("env_ready_srv")
				except rospy.exceptions.ROSInterruptException:
					print("\nCtrl+C is pressed!")
					return self.observation, 0.0, False, {}

				print("Set start_planner to true for to start one execution!")
				rospy.set_param("/ramp/start_planner", True)
				start_waiting_time = rospy.get_rostime()
				print("Wait for this execution completes......")

		if rospy.core.is_shutdown():
			return self.observation, 0.0, False, {}
		
		print("A execution completes!")
		reward = self.reward
		self.reward = None # clear self.reward after it is used

		if self.observation[0].item() > 0.0 and self.observation[0].item() < 1.0:
			ob_can_change = True
		elif self.observation[1].item() > 0.0 and self.observation[1].item() < 1.0:
			ob_can_change = True
		else:
			ob_can_change = False

		if not ob_can_change:
			done = True
			reward = -5000.0
		else:
			done = False

		return self.observation, reward, done, {}