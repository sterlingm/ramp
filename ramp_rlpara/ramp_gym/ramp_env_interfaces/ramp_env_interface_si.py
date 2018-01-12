'''
This is not the environment itself but the interface of environment.
Observation is a single motion state whose size is 10
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

class RampEnv(gym.Env):



    def oneExeInfoCallback(self, data):
        if self.this_exe_info is not None:
            ## TODO: log into file with date in its name
            print(Fore.RED + "Unused RampEnv.this_exe_info is overlaped!")
        self.this_exe_info = data



    def __init__(self):
        self._seed()

        self.check_exe_rate = rospy.Rate(10) # 0.1s
        
        ## get various parameters
        self.utility = Utility()

        a0 = self.utility.coe_A_range[0]
        a1 = self.utility.coe_A_range[1]
        d0 = self.utility.coe_D_range[0]
        d1 = self.utility.coe_D_range[1]

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
        
        self.action_space = spaces.Box(np.array([a0, d0]),
                                       np.array([a1, d1]))
        self.observation_space = spaces.Box(np.array([t0, x0, y0, theta0, x_d0, y_d0, theta_d0, x_dd0, y_dd0, theta_dd0]),
                                            np.array([t1, x1, y1, theta1, x_d1, y_d1, theta_d1, x_dd1, y_dd1, theta_dd1]))
        self.this_exe_info = None
        self.exe_info = None

        self.one_exe_info_sub = rospy.Subscriber("ramp_collection_ramp_ob_one_run", RampObservationOneRunning,
                                                 self.oneExeInfoCallback)



    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]



    def oneCycle(self):
        """Wait for ready, start one execution and wait for its completion.

        Returns
        -------
            Whether this cycle succeeds or not.
        """
        ## wait the actual environment to get ready......
        print("Wait the actual environment to get ready......")
        try:
            rospy.wait_for_service("env_ready_srv")
        except rospy.exceptions.ROSInterruptException:
            print("\nCtrl+C is pressed!")
            return False

        print("Start one execution!")
        rospy.set_param("/ramp/start_planner", True)
        
        ## wait for this execution completes......
        has_waited_exe_for = 0 # seconds
        print("Wait for this execution completes......")
        start_waiting_time = rospy.get_rostime()
        while not rospy.core.is_shutdown() and self.this_exe_info is None:
            try:
                self.check_exe_rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                print("\nCtrl+C is pressed!")
                return False

            cur_time = rospy.get_rostime()
            has_waited_exe_for = cur_time.to_sec() - start_waiting_time.to_sec() # seconds
            if has_waited_exe_for >= self.utility.max_exe_time + 20.0: # overtime
                print("Long time no response!")
                print("Wait the actual environment to get ready......")

                try:
                    rospy.wait_for_service("env_ready_srv")
                except rospy.exceptions.ROSInterruptException:
                    print("\nCtrl+C is pressed!")
                    return False

                print("Start one execution!")
                rospy.set_param("/ramp/start_planner", True)
                start_waiting_time = rospy.get_rostime()
                print("Wait for this execution completes......")

        print("A execution completes!")
        self.exe_info = self.this_exe_info # build many observations used for returning
        self.this_exe_info = None # clear self.this_exe_info after it is used

        return True



    def _reset(self):
        """Set all five coefficients to 1.0 and do one execution.

        Returns
        -------
            Multiple single states.
        """
        rospy.set_param("/ramp/eval_weight_T", 1.0)
        rospy.set_param("/ramp/eval_weight_A", 1.0)
        rospy.set_param("/ramp/eval_weight_D", 1.0)
        rospy.set_param("/ramp/eval_weight_Qc", 1.0)
        rospy.set_param("/ramp/eval_weight_Qk", 1.0)

        if self.oneCycle():
            return self.exe_info.best_trajectory_vector
        else:
            return RampObservationOneRunning().best_trajectory_vector



    def _step(self, action):
        ## Do not limit!
        action = np.clip(action, self.action_space.low, self.action_space.high)

        ## set the coefficients of RAMP
        rospy.set_param("/ramp/eval_weight_A", action[0].item())
        rospy.set_param("/ramp/eval_weight_D", action[1].item())

        if not self.oneCycle():
            return RampObservationOneRunning().best_trajectory_vector, 0.0, False, {'actual_exe_time': self.utility.max_exe_time}
        else:
            ## calculate reward
            reward = self.utility.max_exe_time - self.exe_info.execution_time
            reward = max(0, reward)

            ## done or not
            done = self.exe_info.done

            ## reward and done are both for the last observation in this execution
            return self.exe_info.best_trajectory_vector, reward, done, {'actual_exe_time': self.exe_info.execution_time}