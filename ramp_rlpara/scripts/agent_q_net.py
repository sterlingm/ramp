#!/usr/bin/env python

import os
import rospy
import math
import random
import time
import numpy as np
import tensorflow as tf
import tensorlayer as tl
from tensorlayer.layers import *
import matplotlib.pyplot as plt
from ramp_msgs.srv import *
from ramp_msgs.msg import ParameterUpdates
from ramp_msgs.msg import RampTrajectory

my_inf = 999999.9

## parameters
net_learn_rate = 0.001
goal_radius = 1.5 # unit: meter
traj_fixed_len = 20 # The length of trajectory is variable. Here we limit it to limit the size of input vector.
motion_state_dim = 2*3 # Only use position and velocity (both include orientation) temporarily for simplity
# TODO: add the path of knot points into the observation of RL
# TODO: add some sensing results into the observation of RL
actions_num = 3*3*3 # discretized for simplity temporarily
paras_weight = 40
paras_num = 3
lambd = 0.9    # decay factor
hidden_n_units = 2 * (traj_fixed_len * motion_state_dim + paras_num * paras_weight)
fail_penality = -20.0
preset_T = 1.75
preset_D = 15.0
preset_A = 0.5
penalize_num = 50

env_service_name = 'response2para_update'
node_name = 'agent_q_net'
home_dir = os.getenv("HOME")
fo = open(home_dir + "/data/ramp/ramp_rlpara/random_action.txt", "w")
f_s_a = open(home_dir + "/data/ramp/ramp_rlpara/s_a.txt", "w")
f_all_q = open(home_dir + "/data/ramp/ramp_rlpara/all_q.txt", "w")
f_episode = open(home_dir + "/data/ramp/ramp_rlpara/episode.txt", "w")
do_nothing = ParameterUpdates() # default values are all 0

rospy.init_node(node_name, anonymous=True)
delta_paras_pub = rospy.Publisher('delta_paras', ParameterUpdates, queue_size=10)

wait_0p1_s = rospy.Rate(10) # 10Hz

def getTrajectoryLength(t):
	traj_pts_num = len(t.trajectory.points)
	traj_len = 0.0;
	for i in range(traj_pts_num - 1):
		x1 = t.trajectory.points[i].positions[0]
		y1 = t.trajectory.points[i].positions[1]
		x2 = t.trajectory.points[i+1].positions[0]
		y2 = t.trajectory.points[i+1].positions[1]
		dx = x1 - x2
		dy = y1 - y2
		traj_len += math.sqrt(dx*dx + dy*dy)
	return traj_len

def buildAction(dt, dd, do, dr):
	act = ParameterUpdates()
	act.delta_time_weight = dt
	act.delta_obs_dis_weight = dd
	act.delta_ori_chg_weight = do
	act.delta_rho = dr
	return act

## input:  an action code (integer 0~26)
## output: an actionn (ParameterUpdates (0~2, 0~2, 0~2, 0))
def decodeAction(act_code):
	assert (act_code >= 0 and act_code <= 26)

	_T = rospy.get_param("ramp/eval_weight_T") # T
	_A = rospy.get_param("ramp/eval_weight_A") # A
	_D = rospy.get_param("ramp/eval_weight_D") # D
	## orientation changing weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	sz = _A * 0.02
	if sz < 0.001:
		sz = 0.01
	do = mod * sz
	act_code = int(act_code / 3)
	
	## obstacle distance weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	sz = _D * 0.02
	if sz < 0.001:
		sz = 0.01
	dd = mod * sz
	act_code = int(act_code / 3)
	
	## time weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	sz = _T * 0.02
	if sz < 0.001:
		sz = 0.01
	dt = mod * sz
	
	## the penalty weight for infeasible trajectory
	dr = 0.0
	
	return buildAction(dt, dd, do, dr)
	
def isReachGoal(t, g):
	l = len(t.trajectory.points)
	x1 = t.trajectory.points[l-1].positions[0]
	y1 = t.trajectory.points[l-1].positions[1]
	x2 = g[0]
	y2 = g[1]
	dx = x1 - x2
	dy = y1 - y2
	dis = math.sqrt(dx*dx + dy*dy)
	if dis < 0.5:
		return True
	else:
		return False
		
def isReachGoalNewR(t, g):
	l = len(t.trajectory.points)
	x1 = t.trajectory.points[l-1].positions[0]
	y1 = t.trajectory.points[l-1].positions[1]
	x2 = g[0]
	y2 = g[1]
	dx = x1 - x2
	dy = y1 - y2
	dis = math.sqrt(dx*dx + dy*dy)
	if dis < 2.5:
		return True
	else:
		return False

def envStepWait(act):
	delta_paras_pub.publish(act) # change parameters
	wait_1_s = rospy.Rate(1) # 1Hz
	wait_1_s.sleep() # wait the ramp_planner use the new parameters to evolute for a while
	
	## use service to get the newest best trajectory
	rospy.wait_for_service(env_service_name)
	try:
		getBestTrajectory = rospy.ServiceProxy(env_service_name, EnvironmentSrv)
		response = getBestTrajectory(do_nothing)
		best_trajectory = response.trajectory
		goal = rospy.get_param("/robot_info/goal")
		fail_num = 0
		## wait until the best trajectory is ok
		while not isReachGoal(best_trajectory, goal) or not best_trajectory.feasible:
			print("-----the best trajectory don't reach goal or isn't feasible!-----")
			wait_0p2_s = rospy.Rate(5) # 5Hz
			wait_0p2_s.sleep()
			rospy.wait_for_service(env_service_name)
			response = getBestTrajectory(do_nothing)
			best_trajectory = response.trajectory
			fail_num += 1
			if fail_num >= penalize_num:
				break

		if fail_num >= penalize_num:
			print("-----penalize! penalize! penalize!-----")
			len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
			reward = fail_penality
			done = False
		else:
			len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
			reward = -len_best_trajectory
			if len_best_trajectory < goal_radius:
				done = True
			else:
				done = False
				
		robot_state = response.state
		is_success = True
		paras = np.zeros(paras_num, 'float64')
		paras[0] = rospy.get_param("ramp/eval_weight_T") # T
		paras[1] = rospy.get_param("ramp/eval_weight_A") # A
		paras[2] = rospy.get_param("ramp/eval_weight_D") # D

		return robot_state, best_trajectory, paras, reward, done, is_success
	except rospy.ServiceException as exc:
		fo.close()
		f_s_a.close()
		f_all_q.close()
		f_episode.close()
		print("EnvironmentSrv did not process request: " + str(exc))
		
def envStepNewR(act):
	delta_paras_pub.publish(act) # change parameters
	wait_1_s = rospy.Rate(1) # 1Hz
	wait_1_s.sleep() # wait the ramp_planner use the new parameters to evolute for a while
	
	## use service to get the newest best trajectory
	rospy.wait_for_service(env_service_name)
	try:
		getBestTrajectory = rospy.ServiceProxy(env_service_name, EnvironmentSrv)
		response = getBestTrajectory(do_nothing)
		best_trajectory = response.trajectory
		goal = rospy.get_param("/robot_info/goal")
		if isReachGoalNewR(best_trajectory, goal) and best_trajectory.feasible:
			reward = 0
		else:
			reward = -1

		robot_state = response.state
		dx = robot_state.positions[0] - goal[0]
		dy = robot_state.positions[1] - goal[1]
		if math.sqrt(dx*dx + dy*dy) < goal_radius:
			done = True
		else:
			done = False
			
		is_success = True
		paras = np.zeros(paras_num, 'float64')
		paras[0] = rospy.get_param("ramp/eval_weight_T") # T
		paras[1] = rospy.get_param("ramp/eval_weight_A") # A
		paras[2] = rospy.get_param("ramp/eval_weight_D") # D

		return robot_state, best_trajectory, paras, reward, done, is_success
	except rospy.ServiceException as exc:
		fo.close()
		f_s_a.close()
		f_all_q.close()
		f_episode.close()
		print("EnvironmentSrv did not process request: " + str(exc))
		
def envStep(act):
	rospy.wait_for_service(env_service_name)
	try:
		getBestTrajectory = rospy.ServiceProxy(env_service_name, EnvironmentSrv)
		response = getBestTrajectory(act)
		if response.is_valid == True:
			best_trajectory = response.trajectory
			## Is best_trajectory reaches goal?
			goal = rospy.get_param("/robot_info/goal")
			if isReachGoal(best_trajectory, goal) and best_trajectory.feasible:
				len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
				reward = -len_best_trajectory
				if len_best_trajectory < goal_radius:
					done = True
				else:
					done = False
				is_ok = True
			else:
				reward = -my_inf
				done = False
				is_ok = False
				
			is_success = True
		else:
			best_trajectory = RampTrajectory()
			len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
			reward = -len_best_trajectory
			done = False
			is_success = False
			is_ok = False
		return best_trajectory, reward, done, is_success, is_ok
	except rospy.ServiceException as exc:
		fo.close()
		f_s_a.close()
		f_all_q.close()
		f_episode.close()
		print("EnvironmentSrv did not process request: " + str(exc))
		
def buildQNetIn(robs, t, p):
	array = np.zeros(traj_fixed_len * motion_state_dim + paras_num * paras_weight, 'float64')
	for i in range(traj_fixed_len):
		if i < len(t.trajectory.points):
			array[i*motion_state_dim]   = t.trajectory.points[i].positions[0] - robs.positions[0] # x
			array[i*motion_state_dim+1] = t.trajectory.points[i].positions[1] - robs.positions[1] # y
			array[i*motion_state_dim+2] = t.trajectory.points[i].positions[2] - robs.positions[2] # theta
			array[i*motion_state_dim+3] = t.trajectory.points[i].velocities[0] - robs.velocities[0] # x'
			array[i*motion_state_dim+4] = t.trajectory.points[i].velocities[1] - robs.velocities[1] # y'
			array[i*motion_state_dim+5] = t.trajectory.points[i].velocities[2] - robs.velocities[2] # theta'
		else:
			array[i*motion_state_dim]   = 0.0 # x
			array[i*motion_state_dim+1] = 0.0 # y
			array[i*motion_state_dim+2] = 0.0 # theta
			array[i*motion_state_dim+3] = 0.0 # x'
			array[i*motion_state_dim+4] = 0.0 # y'
			array[i*motion_state_dim+5] = 0.0 # theta'
	
	for i in range(paras_weight):
		array[traj_fixed_len * motion_state_dim+i*paras_num] = p[0] # T
		array[traj_fixed_len * motion_state_dim+i*paras_num+1] = p[1] # A
		array[traj_fixed_len * motion_state_dim+i*paras_num+2] = p[2] # D

	return array
	
def act_iden(x):
	return x
	
def act_soft_plus(x):
	return -tf.nn.softplus(x)
	
def qn():
	global penalize_num
	global fail_penality
	tf.reset_default_graph()
	print("Initialize tensorflow successfully!")
	
	## Define Q-network q(a,s) that ouput the rewards of 27 actions by given state.
	## The actions are discretized for simplity temporarily.
	## Other RL agents, such as DDPG and NAF, can be considered for continuous action space.
	inputs = tf.placeholder(shape=[1, traj_fixed_len * motion_state_dim + paras_num * paras_weight], dtype=tf.float32)
	net = InputLayer(inputs, name='observation')
	#net = DropoutLayer(net, keep=0.8, name='drop1')
	net = DenseLayer(net, n_units=hidden_n_units, act=act_soft_plus,
		W_init=tf.random_uniform_initializer(-0.01, 0.01), b_init=None, name='hidden')
	#net = DropoutLayer(net, keep=0.5, name='drop2')
	net = DenseLayer(net, n_units=actions_num, act=act_soft_plus,
		W_init=tf.random_uniform_initializer(-0.01, 0.01), b_init=None, name='q_a_s') # output layer
	outputs_gragh = net.outputs # action-value / rewards of 27 actions
	predict = tf.argmax(outputs_gragh, 1) # chose action greedily with reward. In Q-Learning, policy is greedy, so we use "max" to select the next action.
	print("Define Q-network q(a,s) successfully!")
	
	## Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
	nextQ = tf.placeholder(shape=[1, actions_num], dtype=tf.float32)
	loss = tl.cost.mean_squared_error(nextQ, outputs_gragh, is_mean = False) # tf.reduce_sum(tf.square(nextQ - y))
	train_op = tf.train.GradientDescentOptimizer(learning_rate = net_learn_rate).minimize(loss)
	print("Define train operator successfully!")
	
	## parameter preset
	rospy.set_param("ramp/eval_weight_T", preset_T)
	rospy.set_param("ramp/eval_weight_D", preset_D)
	rospy.set_param("ramp/eval_weight_A", preset_A)
	e = 0.3 # e-Greedy Exploration, the larger the more random
	
	# wait
	wait_1_s = rospy.Rate(1) # 1Hz
	wait_1_s.sleep() # 1.0s
	
	last_start_flag = False
	start_flag = False
	print("start session!")
	with tf.Session() as sess:
		tl.layers.initialize_global_variables(sess)
		i_episode = 0
		while True: # episode
			## wait a new episode
			last_start_flag = start_flag
			start_flag = rospy.get_param("ramp/start_planner")
			while not (last_start_flag == False and start_flag == True):
				if rospy.has_param("rl_done"):
					rospy.delete_param("rl_done")
				last_start_flag = start_flag
				start_flag = rospy.get_param("ramp/start_planner")
				print("RL agent is waiting a new episode......")
				time.sleep(0.5) # 0.5s
			
			## start episode
			print("start episode!")
			i_episode += 1
			episode_time = time.time()
			robs, s, paras, _, d, _ = envStepWait(do_nothing) # do nothing, just get an initial observation
			rAll = 0
			i_step = 0
			while True: # step
				i_step += 1
				## Choose an action by greedily (with e chance of random action) from the Q-network
				ss = buildQNetIn(robs, s, paras)
				a, allQ = sess.run([predict, outputs_gragh], feed_dict={inputs : [ss]})
				f_all_q.write(str(allQ) + "\n")
				## e-Greedy Exploration !!! sample random action
				if np.random.rand(1) < e:
					a[0] = math.floor(np.random.rand(1) * 27)
					print("-----random action!-----")
					fo.write("-----random action!-----\n")
					
				act = decodeAction(a[0])
		        ## Get new state and reward from environment
				robs1, s1, paras1, r, d, _ = envStepWait(act)
				## loa state, reward, done and action
				s_a = robs, ss, r, d, act
				f_s_a.write(str(s_a) + "\n")
				## Obtain the Q' values by feeding the new state through our network
				ss1 = buildQNetIn(robs1, s1, paras1)
				Q1 = sess.run(outputs_gragh, feed_dict={inputs : [ss1]})
				## Obtain maxQ' and set our target value.
				maxQ1 = np.max(Q1)  # in Q-Learning use "max" to set target value.
				targetQ = allQ
				targetQ[0, a[0]] = r + lambd * maxQ1
				## Train network using target and predicted Q values
				# it is not real target Q value, it is just an estimation,
				# but check the Q-Learning update formula:
				#    Q'(s,a) <- Q(s,a) + alpha(r + lambd * maxQ(s',a') - Q(s, a))
				# minimizing |r + lambd * maxQ(s',a') - Q(s, a)|^2 equal to force
				#   Q'(s,a) ≈ Q(s,a)
				_ = sess.run(train_op, {inputs : [ss], nextQ : targetQ})
				f_all_q.write(str(ss) + "\n" + str(targetQ) + "\n")
				rAll += r
				
				robs = robs1
				s = s1
				paras = paras1
				
				# print("done = ", d)
				if rospy.has_param("rl_done"):
					d = rospy.get_param("rl_done")
				if d == True:
					## Reduce chance of random action if an episode is done.
					e -= 0.01
					
					penalize_num += 1
					if penalize_num > 50:
						penalize_num = 50
						
					fail_penality += 10
					if fail_penality > -20:
						fail_penality = -20
						
					break
					
			## log episodic data
			epi_data = rAll, i_step, time.time() - episode_time
			f_episode.write(str(epi_data) + "\n")
				
	print("Exit qn()!")

if __name__ == "__main__":
	try:
		# test()
		qn()
		fo.close()
		f_s_a.close()
		f_all_q.close()
		f_episode.close()
	except rospy.ROSInterruptException:
		## procedure must be closed normally to ensure the files are written normally
		fo.close()
		f_s_a.close()
		f_all_q.close()
		f_episode.close()
		pass
