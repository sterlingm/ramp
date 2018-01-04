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

		self.action_space = spaces.Box(np.array([self.utility.coe_dT_range[0],
		                                         self.utility.coe_dA_range[0],
												 self.utility.coe_dD_range[0],
												 self.utility.coe_dQc_range[0],
												 self.utility.coe_dQk_range[0]]),
		                               np.array([self.utility.coe_dT_range[1],
									             self.utility.coe_dA_range[1],
												 self.utility.coe_dD_range[1],
												 self.utility.coe_dQc_range[1],
												 self.utility.coe_dQk_range[1]]))

		self.observation_space = spaces.Box(np.array([self.utility.coe_T_range[0],
		                                              self.utility.coe_A_range[0],
													  self.utility.coe_D_range[0],
													  self.utility.coe_Qc_range[0],
													  self.utility.coe_Qk_range[0]]),
		                                    np.array([self.utility.coe_T_range[1],
											          self.utility.coe_A_range[1],
													  self.utility.coe_D_range[1],
													  self.utility.coe_Qc_range[1],
													  self.utility.coe_Qk_range[1]]))
		self.reward = None

		self.reward_sub = rospy.Subscriber("ramp_collection_reward_offline", Float64, self.rewardCallback)



	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]
		


	def _reset(self):
		## set coefficients randomly
		# observation = self.observation_space.sample()

		observation = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
		rospy.set_param("/ramp/eval_weight_T", observation[0].item())
		rospy.set_param("/ramp/eval_weight_A", observation[1].item())
		rospy.set_param("/ramp/eval_weight_D", observation[2].item())
		rospy.set_param("/ramp/eval_weight_Qc", observation[3].item())
		rospy.set_param("/ramp/eval_weight_Qk", observation[4].item())
		
		return observation



	def _step(self, action):
		assert self.action_space.contains(action)

		## read the coefficients of RAMP
		wT = rospy.get_param("/ramp/eval_weight_T")
		wA = rospy.get_param("/ramp/eval_weight_A")
		wD = rospy.get_param("/ramp/eval_weight_D")
		wQc = rospy.get_param("/ramp/eval_weight_Qc")
		wQk = rospy.get_param("/ramp/eval_weight_Qk")

		## build a observation
		observation = np.array([wT, wA, wD, wQc, wQk])
		assert self.observation_space.contains(observation)
		observation += action
		
		## limit the coefficients (observation vector) into the observation space
		observation = np.clip(observation,
		                      self.observation_space.low,
							  self.observation_space.high)
		
		## change the coefficients of RAMP
		rospy.set_param("/ramp/eval_weight_T", observation[0].item())
		rospy.set_param("/ramp/eval_weight_A", observation[1].item())
		rospy.set_param("/ramp/eval_weight_D", observation[2].item())
		rospy.set_param("/ramp/eval_weight_Qc", observation[3].item())
		rospy.set_param("/ramp/eval_weight_Qk", observation[4].item())
		
		## wait the actual environment to get ready......
		print("Wait the actual environment to get ready......")
		try:
			rospy.wait_for_service("env_ready_srv")
		except rospy.exceptions.ROSInterruptException:
			print("\nCtrl+C is pressed!")
			return observation, 0.0, False, {}

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
					return observation, 0.0, False, {}

				print("Set start_planner to true for to start one execution!")
				rospy.set_param("/ramp/start_planner", True)
				start_waiting_time = rospy.get_rostime()
				print("Wait for this execution completes......")

		if rospy.core.is_shutdown():
			return observation, 0.0, False, {}
		
		print("A execution completes!")
		reward = self.reward
		self.reward = None # clear self.reward after it is used

		return observation, reward, False, {}