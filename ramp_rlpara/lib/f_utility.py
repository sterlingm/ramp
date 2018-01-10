import numpy as np
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

class Utility(object):

    def __init__(self):
        ## TODO: if there is parameter not being set, give warning!
        self.replay_buffer_size = rospy.get_param('/replay_buffer_size', 100000) # if get fail, use default value
        self.discount_factor = rospy.get_param('/discount_factor', 0.9)
        self.target_net_update_factor = rospy.get_param('/target_net_update_factor', 0.001) # big inertia
        self.mini_batch_size = rospy.get_param('/mini_batch_size', 32)
        self.max_exe_time = rospy.get_param('/max_exe_time', 60.0) # seconds
        self.switch_period = rospy.get_param('/ramp/fixed_control_cycle_rate', 1.2) # seconds
        self.the_initial = rospy.get_param('/robot_info/start', [0.0, 0.0, 0.785])
        self.orn_paras = rospy.get_param('/ornstein_uhlenbeck_paras', {'theta': 10.0, 'percent': 0.33333})
        self.max_nb_exe = rospy.get_param('/max_nb_exe', 50000)
        self.max_episode_steps = rospy.get_param('/max_episode_steps', 100)
        self.nb_steps_warmup_critic = rospy.get_param('/nb_steps_warmup_critic', 0)
        self.nb_steps_warmup_actor = rospy.get_param('/nb_steps_warmup_actor', 0)
        self.critic_lr = rospy.get_param('/critic_lr', 0.001)
        self.gradient_clip_norm = rospy.get_param('/gradient_clip_norm', 0.5)

        self.coe_dT_range = rospy.get_param('/coe_dT_range', [-0.01, 0.01])
        self.coe_dA_range = rospy.get_param('/coe_dA_range', [-0.01, 0.01])
        self.coe_dD_range = rospy.get_param('/coe_dD_range', [-0.01, 0.01])
        self.coe_dQc_range = rospy.get_param('/coe_dQc_range', [-0.01, 0.01])
        self.coe_dQk_range = rospy.get_param('/coe_dQk_range', [-0.01, 0.01])
        
        self.coe_T_range = rospy.get_param('/coe_T_range', [0.0, 1.0])
        self.coe_A_range = rospy.get_param('/coe_A_range', [0.0, 1.0])
        self.coe_D_range = rospy.get_param('/coe_D_range', [0.0, 1.0])
        self.coe_Qc_range = rospy.get_param('/coe_Qc_range', [0.0, 1.0])
        self.coe_Qk_range = rospy.get_param('/coe_Qk_range', [0.0, 1.0])

        ## max time stamp of motion state in the best trajectory
        self.time_stamp_max = rospy.get_param('/time_stamp_max', 25.0) # seconds
        assert self.time_stamp_max > 0

        ## minimal x, y and theta
        dof_min = rospy.get_param('/robot_info/DOF_min', [0.0, 0.0, -math.pi])
        self.min_x = dof_min[0] - 0.6
        self.min_y = dof_min[1] - 0.6
        self.min_theta = dof_min[2]

        ## maximal x, y and theta
        dof_max = rospy.get_param('/robot_info/DOF_max', [3.5, 3.5, math.pi])
        self.max_x = dof_max[0] + 0.05
        self.max_y = dof_max[1] + 0.05
        self.max_theta = dof_max[2]

        ## maximal velocity, linear and angular, both absolute
        self.max_linear_v = rospy.get_param('/robot_info/max_speed_linear', 0.33)
        self.max_angular_v = rospy.get_param('/robot_info/max_speed_angular', 1.5708)

        ## maximal acceleration, linear and angular, both absolute
        self.max_linear_a = rospy.get_param('/robot_info/max_acceleration_linear', 2.0)
        self.max_angular_a = rospy.get_param('/robot_info/max_acceleration_angular', 1.0)



    def normalizeMotionState(self, s): # s is a np.array whose shape is (10,)
        assert len(s) == 10

        ## normalize time stamp of motion state
        t = s[0]
        t = np.clip(t, 0.0, self.time_stamp_max) # clip using non-normalized value
        t_normed = t / self.time_stamp_max # time stamp, second

        ## normalize x, y of motion state
        x = s[1]
        y = s[2]

        x = np.clip(x, self.min_x, self.max_x)
        y = np.clip(y, self.min_y, self.max_y)

        x_normed = (x - self.min_x) / (self.max_x - self.min_x)
        y_normed = (y - self.min_y) / (self.max_y - self.min_y)

        ## normalize theta of motion state
        #  after normalizing, 0.0 is equivalent to 1.0. this is OK,
        #  because [0, 1] only represent different position angle,
        #  there is no size relation between different angle
        theta = s[3]
        while theta < self.min_theta:
            theta += 2 * math.pi
        while theta >= self.max_theta:
            theta -= 2 * math.pi
        theta_normed = (theta - self.min_theta) / (self.max_theta - self.min_theta)

        ## normalize x_dot, y_dot and theta_dot
        x_dot = s[4]
        y_dot = s[5]
        theta_dot = s[6]

        x_dot = np.clip(x_dot, -self.max_linear_v, self.max_linear_v)
        y_dot = np.clip(y_dot, -self.max_linear_v, self.max_linear_v)
        theta_dot = np.clip(theta_dot, -self.max_angular_v, self.max_angular_v)

        x_dot_normed = (x_dot - (-self.max_linear_v)) / (self.max_linear_v - (-self.max_linear_v))
        y_dot_normed = (y_dot - (-self.max_linear_v)) / (self.max_linear_v - (-self.max_linear_v))
        theta_dot_normed = (theta_dot - (-self.max_angular_v)) / (self.max_angular_v - (-self.max_angular_v))

        ## normalize x_dd, y_dd and theta_dd
        x_dd = s[7]
        y_dd = s[8]
        theta_dd = s[9]

        x_dd = np.clip(x_dd, -self.max_linear_a, self.max_linear_a)
        y_dd = np.clip(y_dd, -self.max_linear_a, self.max_linear_a)
        theta_dd = np.clip(theta_dd, -self.max_angular_a, self.max_angular_a)

        x_dd_normed = (x_dd - (-self.max_linear_a)) / (self.max_linear_a - (-self.max_linear_a))
        y_dd_normed = (y_dd - (-self.max_linear_a)) / (self.max_linear_a - (-self.max_linear_a))
        theta_dd_normed = (theta_dd - (-self.max_angular_a)) / (self.max_angular_a - (-self.max_angular_a))

        return np.array([t_normed, x_normed, y_normed, theta_normed,
                                   x_dot_normed, y_dot_normed, theta_dot_normed,
                                   x_dd_normed, y_dd_normed, theta_dd_normed])


    def normCoes(self, coes, env):
        coes_normed = coes
        coes_normed = np.clip(coes_normed,
                              env.observation_space.low,
                              env.observation_space.high)

        coes_normed = (coes_normed - env.observation_space.low) / (env.observation_space.high - env.observation_space.low)

        return coes_normed



    def antiNormalizeCoes(self, coes):
        assert len(coes) == 3
        
        A = self.coe_A_range[0] + coes[0] * (self.coe_A_range[1] - self.coe_A_range[0]) # transfer into non-normalized value
        A = np.clip(A, self.coe_A_range[0], self.coe_A_range[1]) # clip using non-normalized value

        D = self.coe_D_range[0] + coes[1] * (self.coe_D_range[1] - self.coe_D_range[0])
        D = np.clip(D, self.coe_D_range[0], self.coe_D_range[1])

        Qk = self.coe_Qk_range[0] + coes[2] * (self.coe_Qk_range[1] - self.coe_Qk_range[0])
        Qk = np.clip(Qk, self.coe_Qk_range[0], self.coe_Qk_range[1])

        return np.array([A, D, Qk])



    def normDeltaCoes(self, delta_coes, env):
        delta_coes_normed = delta_coes
        delta_coes_normed = np.clip(delta_coes_normed,
                                    env.action_space.low,
                                    env.action_space.high)

        delta_coes_normed = (delta_coes_normed - env.action_space.low) / (env.action_space.high - env.action_space.low)

        return delta_coes_normed

    def antiNormDeltaCoes(self, delta_coes_normed, env):
        delta_coes = delta_coes_normed
        delta_coes = np.clip(delta_coes, 0.0, 1.0)

        delta_coes = env.action_space.low + delta_coes * (env.action_space.high - env.action_space.low)

        return delta_coes



    ## TODO: implement it
    # def nparray2Float64MultiArray(np_arr)
    #     shape = np_arr.shape
    #     multi_arr = Float64MultiArray()

    #     axis_id = 0
    #     for axis_sz in shape:
    #         axis = MultiArrayDimension('axis_' + str(axis_id), axis_sz)
    #         multi_arr.layout.dim.append(axis)
    #         axis_id += 1

    #     bigger_id_stride = 1
    #     for axis_id in range(len(multi_arr.layout.dim)-1, -1, -1): # inverse for loop
    #         multi_arr.layout.dim[axis_id].stride = multi_arr.layout.dim[axis_id].size * bigger_id_stride
    #         bigger_id_stride = multi_arr.layout.dim.stride

    #     return multi_arr