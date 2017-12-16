'''
This is not the environment itself but the interface of environment.
Observation is a single motion state whose size is 10
'''

import time
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import rospy
import warnings
from std_msgs.msg import Float64
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

	def setEnvRdyTrueCallback(self, data):
		self.env_ready = True

	def oneExeInfoCallback(self, data)
		if self.this_exe_info is not None:
			## TODO: log into file with date in its name
			warnings.warn("Unused RampEnv.this_exe_info is overlaped!")
		self.this_exe_info = data

	def __init__(self):
		## get various parameters
		self.utility = Utility()

		a0 = self.utility.coe_A_range[0]
		a1 = self.utility.coe_A_range[1]

		d0 = self.utility.coe_D_range[0]
		d1 = self.utility.coe_D_range[1]

		qk0 = self.utility.coe_Qk_range[0]
		qk1 = self.utility.coe_Qk_range[1]

		t0 = 0.0
		t1 = self.utility.time_stamp_max

		x0 = self.utility.min_x
		x1 = self.utility.max_x

		y0 = self.utility.min_y
		y1 = self.utility.max_y

		theta0 = self.utility.min_theta
		theta1 = self.utility.max_theta

		x_d0 = -self.utility.max_linear_v
		x_d1 =  self.utility.max_linear_v

		y_d0 = -self.utility.max_linear_v
		y_d1 =  self.utility.max_linear_v

		theta_d0 = -self.utility.max_angular_v
		theta_d1 =  self.utility.max_angular_v

		x_dd0 = -self.utility.max_linear_a
		x_dd1 =  self.utility.max_linear_a

		y_dd0 = -self.utility.max_linear_a
		y_dd1 =  self.utility.max_linear_a

		theta_dd0 = -self.utility.max_angular_a
		theta_dd1 =  self.utility.max_angular_a
		
		self.action_space = spaces.Box(np.array([a0, d0, qk0]),
		                               np.array([a1, d1, qk1]))
		self.observation_space = spaces.Box(np.array([t0, x0, y0, theta0, x_d0, y_d0, theta_d0, x_dd0, y_dd0, theta_dd0]),
		                                    np.array([t1, x1, y1, theta1, x_d1, y_d1, theta_d1, x_dd1, y_dd1, theta_dd1]))
		self.set_env_rdy_true_sub = rospy.Subscriber("set_env_ready_true", Empty, self.setEnvRdyTrueCallback)
		self.one_exe_info_sub = rospy.Subscriber("ramp_collection_ramp_ob_one_run", RampObservationOneRunning, self.oneExeInfoCallback)
		self.env_ready = False
		self.this_exe_info = None

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

		## set the coefficients of RAMP
		rospy.set_param("/ramp/eval_weight_A", action[0])
		rospy.set_param("/ramp/eval_weight_D", action[1])
		rospy.set_param("/ramp/eval_weight_Qk", action[2])

		## wait the actual environment to get ready......
		print("wait the actual environment to get ready......")
		while not self.env_ready:
			time.sleep(0.02) # 0.02s
		print("find env. ready and set start_planner True for the ready env. to start one execution!")
		rospy.set_param("/ramp/start_planner", True)
		self.env_ready = False
		
		## wait for this execution completes......
		print("wait for this execution completes......")
		while self.this_exe_info is None: # TODO: enable key interrupt
			time.sleep(0.1) # 0.1s
		print("A execution completes!")
		observations = self.this_exe_info # build many observations used for returning
		self.this_exe_info = None # clear self.this_exe_info after it is used

		## calculate reward
		reward = self.utility.max_exe_time - self.this_exe_info.execution_time
		reward = max(0, reward)

		## done or not
		done = self.this_exe_info.done

		## reward and done are both for the last observation in this execution
		return observations, reward, done, {}
