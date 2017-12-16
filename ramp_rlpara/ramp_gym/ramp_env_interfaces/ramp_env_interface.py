'''
This is not the environment itself but the interface of environment.
Observation is (A, D, Qk)
'''

import time
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import rospy
from collections import deque
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from ramp_msgs.msg import RampObservationOneRunning

class RampEnv(gym.Env):

	def exeTimeCallback(self, data):
		assert data.data > 0
		assert len(self.exe_time_vector) < self.nb_runs_one_step
		self.exe_time_vector = np.append(self.exe_time_vector, data.data)

	def setEnvRdyTrueCallback(self, data):
		self.env_ready = True

	def __init__(self):
		self.action_space = spaces.Box(np.array([-0.003, -3.0, -3.0]), np.array([0.003, 3.0, 3.0]))
		self.observation_space = spaces.Box(np.array([0.0, 0.0, 0.0]), np.array([0.05, 50.0, 50.0]))
		self.last_calcu_exe_time = -1.0 # init a invalid value
		self.nb_runs_one_step = 3
		self.exe_time_vector = np.array([])
		self.len_observation_buffer = 15 # TODO: buffer length can be parameterized
		self.observation_dimension = self.observation_space.shape[0]
		self.observation_buffer = deque(maxlen = self.len_observation_buffer)
		self.exe_time_sub = rospy.Subscriber("execution_time", Float64, self.exeTimeCallback)
		self.set_env_rdy_true_sub = rospy.Subscriber("set_env_ready_true", Empty, self.setEnvRdyTrueCallback)
		self.env_ready = False

		self._seed()

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]
		
	def _reset(self):
		## set parameters randomly
		observation = self.observation_space.sample()
		rospy.set_param("/ramp/eval_weight_A", observation[0].item())
		rospy.set_param("/ramp/eval_weight_D", observation[1].item())
		rospy.set_param("/ramp/eval_weight_Qk", observation[2].item())

		## the robot start go and it will make many runnings (one running one execution time)
		# rospy.set_param("/start_one_step", True)
		
		## wait for the robot to complete all runnings and return a observation_one_running vector
		print("wait for the robot to complete all runnings......")
		while len(self.exe_time_vector) != self.nb_runs_one_step:
			print("wait the actual environment to get ready......")
			while not self.env_ready:
				time.sleep(0.5) # 0.5s
			print("set start_planner True for the ready environment to start one running!")
			rospy.set_param("/ramp/start_planner", True)
			self.env_ready = False
		print("initial running has been completed!")
		# print("execution times of initial running: " + str(self.exe_time_vector) + " seconds")

		## prevent the robot start go
		# rospy.set_param("/start_one_step", False)

		## process execution time vector (try to make RAMP success more)
		#  TODO: maybe need improvement
		etv = self.exe_time_vector
		self.exe_time_vector = np.array([]) # clear self.exe_time_vector after it is used
		assert len(etv) == self.nb_runs_one_step
		assert len(etv) >= 1
		if len(etv) >= 3:
			etv = np.delete(etv, [etv.argmin(), etv.argmax()]) # delete the min and the max in the vector
		self.last_calcu_exe_time = etv.mean() # the execution time used for calculating reward
		assert self.last_calcu_exe_time > 0

		return observation

	def _step(self, action):
		assert self.action_space.contains(action)
	
		## read the parameters of RAMP
		wA = rospy.get_param("/ramp/eval_weight_A")
		wD = rospy.get_param("/ramp/eval_weight_D")
		wQk = rospy.get_param("/ramp/eval_weight_Qk")
		wA += action[0].item()
		wD += action[1].item()
		wQk += action[2].item()
		
		## limit the parameters (observation vector) into the observation space
		assert self.observation_dimension == 3
		low = self.observation_space.low
		high = self.observation_space.high
		wA_min = low[0].item()
		wA_max = high[0].item()
		wD_min = low[1].item()
		wD_max = high[1].item()
		wQk_min = low[2].item()
		wQk_max = high[2].item()
		if wA < wA_min:
			wA = wA_min
		if wA > wA_max:
			wA = wA_max
		if wD < wD_min:
			wD = wD_min
		if wD > wD_max:
			wD = wD_max
		if wQk < wQk_min:
			wQk = wQk_min
		if wQk > wQk_max:
			wQk = wQk_max
		
		## change the parameters of RAMP
		rospy.set_param("/ramp/eval_weight_A", wA)
		rospy.set_param("/ramp/eval_weight_D", wD)
		rospy.set_param("/ramp/eval_weight_Qk", wQk)
		
		## build a observation used for returning
		observation = np.array([wA, wD, wQk])
		assert self.observation_space.contains(observation)
		
		## buffer observation
		self.observation_buffer.append(observation) # remove the oldest observation autonomously
		
		## start a new step
		#  the robot start go and it will make many runnings (one running one execution time)
		# rospy.set_param("/start_one_step", True)
		
		## wait for the robot to complete all runnings and return a execution time vector
		print("wait for the robot to complete all runnings......")
		while len(self.exe_time_vector) != self.nb_runs_one_step: # TODO: enable key interrupt
			print("wait the actual environment to get ready......")
			while not self.env_ready:
				time.sleep(0.5) # 0.5s
			print("set start_planner True for the ready environment to start one running!")
			rospy.set_param("/ramp/start_planner", True)
			self.env_ready = False
		print("one step has been completed!")
		# print("execution times of this step: " + str(self.exe_time_vector) + " seconds")
		
		## prevent the robot go before starting a new step
		# rospy.set_param("/start_one_step", False)

		## process execution time vector (try to make RAMP success more)
		#  TODO: maybe need improvement
		exe_time_vector = self.exe_time_vector # returning info needs this line
		self.exe_time_vector = np.array([]) # clear self.exe_time_vector after it is used
		etv = exe_time_vector
		assert len(etv) == self.nb_runs_one_step
		assert len(etv) >= 1
		if len(etv) >= 3:
			etv = np.delete(etv, [etv.argmin(), etv.argmax()]) # delete the min and the max in the vector
		calcu_exe_time = etv.mean() # the execution time used for calculating reward
		assert calcu_exe_time > 0

		## calculate reward
		delta_calcu_exe_time = calcu_exe_time - self.last_calcu_exe_time
		self.last_calcu_exe_time = calcu_exe_time
		reward = -delta_calcu_exe_time

		## TODO: done or not. TODO: maybe need improvement
		if len(self.observation_buffer) == self.observation_buffer.maxlen:
			# when there is enough observations
			# done = True or done = False
			done = False # just used for returning
		else:
			done = False

		return observation, reward, done, {"execution_time_vector": exe_time_vector, "calculated_execution_time": calcu_exe_time}