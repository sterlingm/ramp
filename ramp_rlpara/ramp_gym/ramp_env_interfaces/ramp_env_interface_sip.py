'''
This is not the environment itself but the interface of environment.
Observation is a path and its corresponding coefficients.
'''

import os
import sys
import time
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from ramp_msgs.msg import RampTrajectory
from ramp_msgs.msg import RampObservationOneRunning
from colorama import init as clr_ama_init
from colorama import Fore
clr_ama_init(autoreset = True)

## get directory
rlpara_root = os.path.join(os.path.dirname(__file__), '../../')
lib_dir = os.path.join(rlpara_root, 'lib/')
sys.path.append(lib_dir)

## from .py_file_name import class_name
from f_utility import Utility

class RampEnvSip(gym.Env):



    def oneExeInfoCallback(self, data):
        if self.this_exe_info is not None:
            print(Fore.RED + "Unused this_exe_info is overlaped!")
        self.this_exe_info = data



    def __init__(self, name='ramp_sip'):
        self.name = name
        self.check_exe_rate = rospy.Rate(10) # 0.1s
        
        ## get various parameters
        self.utility = Utility()
        self.action_resolution = 0.05
        self.max_obs_cost = 0.9
        self.max_time_cost = 33.0

        self.a0 = self.utility.coe_A_range[0]
        self.a1 = self.utility.coe_A_range[1]
        self.d0 = self.utility.coe_D_range[0]
        self.d1 = self.utility.coe_D_range[1]
        
        self.action_space = spaces.Discrete(9) # 3 * 3 = 9
        self.observation_space = spaces.Box(np.array([self.utility.min_x, self.utility.min_y, self.a0, self.d0]),
                                            np.array([self.utility.max_x, self.utility.max_y, self.a1, self.d1])) # single motion state
        self.this_exe_info = None
        self.exe_info = None

        self.one_exe_info_sub = rospy.Subscriber("ramp_collection_ramp_ob_one_run", RampObservationOneRunning,
                                                 self.oneExeInfoCallback)



    def oneCycle(self):
        """Wait for ready, start one execution and wait for its completion.

        Returns
        -------
            Whether this cycle succeeds or not.
        """
        ## Wait environment get ready......
        print("Wait environment get ready......")
        try:
            rospy.wait_for_service("env_ready_srv")
        except rospy.exceptions.ROSInterruptException:
            print("\nCtrl+C is pressed!")
            return False

        print("Start one execution!")
        rospy.set_param("/ramp/start_planner", True)
        
        ## Wait execution complete......
        has_waited_exe_for = 0
        print("Wait execution complete......")
        start_waiting_time = rospy.get_rostime()
        while not rospy.core.is_shutdown() and self.this_exe_info is None:
            try:
                self.check_exe_rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                print("\nCtrl+C is pressed!")
                return False

            cur_time = rospy.get_rostime()
            has_waited_exe_for = cur_time.to_sec() - start_waiting_time.to_sec()
            if has_waited_exe_for >= self.utility.max_exe_time + 150.0: # overtime
                print("Long time no response!")
                print("Wait environment get ready......")

                try:
                    rospy.wait_for_service("env_ready_srv")
                except rospy.exceptions.ROSInterruptException:
                    print("\nCtrl+C is pressed!")
                    return False

                print("Start one execution!")
                rospy.set_param("/ramp/start_planner", True)
                start_waiting_time = rospy.get_rostime()
                print("Wait execution complete......")

        print("A execution completes!")
        self.exe_info = self.this_exe_info # Store
        self.this_exe_info = None # Clear self.this_exe_info after it is used

        return True



    def reset(self, full_rand=True):
        """Set two coefficients to zero and do one execution.

        Returns
        -------
            Multiple single states.
        """
        coes = np.random.rand(2)
        coes[0] *= self.a1
        coes[1] *= self.d1
        # coes = np.array([0.0, 0.0])
        self.setState(coes[0], coes[1])

        self.oneCycle()
        return self.getOb(), coes



    def decodeAction(self, action):
        """
        Arguments
        ---------
            action (int): encoded delta A, D weight.

        Return
        ------
            (float): Delta A, D weight.
        """
        if action == 0:
            dA = 0
            dD = 0
        elif action == 1:
            dA = 0
            dD = 1
        elif action == 2:
            dA = 0
            dD = 2
        elif action == 3:
            dA = 1
            dD = 0
        elif action == 4:
            dA = 1
            dD = 1
        elif action == 5:
            dA = 1
            dD = 2
        elif action == 6:
            dA = 2
            dD = 0
        elif action == 7:
            dA = 2
            dD = 1
        elif action == 8:
            dA = 2
            dD = 2

        dA = (dA - 1) * self.action_resolution
        dD = (dD - 1) * self.action_resolution

        return dA, dD



    def _step(self, action):
        print('################################################################')
        dA, dD = self.decodeAction(action)

        ## set the coefficients of RAMP
        A = rospy.get_param('/ramp/eval_weight_A')
        D = rospy.get_param('/ramp/eval_weight_D')
        self.setState(A+dA, D+dD)

        self.oneCycle()
        # Reward are for the whole path and its coefficients.
        return self.getOb(), self.getReward(), False, self.getInfo()



    def setState(self, A, D):
        A = np.clip(A, self.a0, self.a1)
        D = np.clip(D, self.d0, self.d1)
        rospy.set_param('/ramp/eval_weight_A', A.item())
        rospy.set_param('/ramp/eval_weight_D', D.item())



    def getReward(self):
        time_cost = self.exe_info.execution_time
        time_reward = self.max_time_cost - time_cost
        time_reward = max(0, time_reward)
        time_reward *= 0.3

        if self.exe_info.best_trajectory_vector[0].min_obs_dis < 0.1:
            obs_cost = self.max_obs_cost
        else:
            obs_cost = 1.0 / self.exe_info.best_trajectory_vector[0].min_obs_dis
        obs_reward = self.max_obs_cost - obs_cost
        obs_reward = max(0, obs_reward)
        obs_reward *= 5.0

        reward = min(time_reward, obs_reward)
        reward *= 1.5
        return reward



    def getOb(self):
        A = rospy.get_param('/ramp/eval_weight_A')
        D = rospy.get_param('/ramp/eval_weight_D')

        if self.exe_info is None or len(self.exe_info.best_trajectory_vector) == 0:
            return np.array([[0.0, 0.0, A, D]])

        x1 = self.exe_info.best_trajectory_vector[0].holonomic_path.points[1].motionState.positions[0]
        y1 = self.exe_info.best_trajectory_vector[0].holonomic_path.points[1].motionState.positions[1]
        ob = np.array([[x1, y1, A, D]])
        length = len(self.exe_info.best_trajectory_vector[0].holonomic_path.points)
        for i in range(2, length):
            xi = self.exe_info.best_trajectory_vector[0].holonomic_path.points[i].motionState.positions[0]
            yi = self.exe_info.best_trajectory_vector[0].holonomic_path.points[i].motionState.positions[1]
            ob = np.concatenate((ob, [[xi, yi, A, D]]))

        return ob



    def getInfo(self):
        return {'time': self.exe_info.execution_time,
                'obs_dis': self.exe_info.best_trajectory_vector[0].min_obs_dis}