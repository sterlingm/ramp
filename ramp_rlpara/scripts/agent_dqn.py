#!/usr/bin/env python

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

## parameters
goal_radius = 0.8 # unit: meter
traj_fixed_len = 20 # The length of trajectory is variable. Here we limit it to limit the size of input vector.
motion_state_dim = 2*3 # Only use position and velocity (both include orientation) temporarily for simplity
# TODO: add the path of knot points into the observation of RL
# TODO: add some sensing results into the observation of RL
actions_num = 3*3*3 # discretized for simplity temporarily
lambd = .9    # decay factor
hidden_n_units = 100
# collision_penality = -1000

env_service_name = 'response2para_update'
node_name = 'agent_q_net'

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

def test():
	i = 0
	rate = rospy.Rate(2) # 2hz
	while not rospy.is_shutdown():
		i += 1
		exam_act = buildAction(-0.01*i, 0.1, -0.2, 0.2)
		rospy.loginfo("[act " + str(i) + "] dt:" + str(exam_act.delta_time_weight) + " dd: " + str(exam_act.delta_obs_dis_weight) + " do: " + str(exam_act.delta_ori_chg_weight) + " dr: " + str(exam_act.delta_rho))
		best_t, reward, done, is_success = envStep(exam_act)
		if is_success == True:
			rospy.loginfo("[res " + str(i) + "] fitness: " + str(best_t.fitness)[:5] + " reward: " + str(reward) + " done: " + str(done))
			getTrajectoryLength(best_t)
		else:
			rospy.loginfo("Taking [action " + str(i) + "] failed!")
			i -= 1 
		rate.sleep()

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

	## orientation changing weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	do = mod * (0.01 / 20.0)
	act_code = int(act_code / 3)
	
	## obstacle distance weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	dd = mod * (15.0 / 20.0)
	act_code = int(act_code / 3)
	
	## time weight
	mod = act_code % 3
	mod -= 1; # -1~+1
	dt = mod * (1.75 / 20.0)
	
	## the penalty weight for infeasible trajectory
	dr = 0.0
	
	return buildAction(dt, dd, do, dr)

def envStep(act):
	rospy.wait_for_service(env_service_name)
	try:
		getBestTrajectory = rospy.ServiceProxy(env_service_name, EnvironmentSrv)
		response = getBestTrajectory(act)
		if response.is_valid == True: # trajectory reach goal
			best_trajectory = response.trajectory
			len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
			reward = -len_best_trajectory
			# if not best_trajectory.feasible
			#	reward += collision_penality
			print("len_best_t = ", len_best_trajectory)
			if len_best_trajectory < goal_radius:
				done = True
			else:
				done = False
			is_success = True
		else:
			best_trajectory = RampTrajectory()
			len_best_trajectory = getTrajectoryLength(best_trajectory) # unit: meter
			reward = -len_best_trajectory
			done = False
			is_success = False
		return best_trajectory, reward, done, is_success
	except rospy.ServiceException as exc:
		print("EnvironmentSrv did not process request: " + str(exc))
		
def buildQNetIn(t):
	array = np.zeros(traj_fixed_len * motion_state_dim, 'float64')
	for i in range(traj_fixed_len):
		if i < len(t.trajectory.points):
			array[i*motion_state_dim]   = t.trajectory.points[i].positions[0] # x
			array[i*motion_state_dim+1] = t.trajectory.points[i].positions[1] # y
			array[i*motion_state_dim+2] = t.trajectory.points[i].positions[2] # theta
			array[i*motion_state_dim+3] = t.trajectory.points[i].velocities[0] # x'
			array[i*motion_state_dim+4] = t.trajectory.points[i].velocities[1] # y'
			array[i*motion_state_dim+5] = t.trajectory.points[i].velocities[2] # theta'
		else:
			array[i*motion_state_dim]   = 0.0 # x
			array[i*motion_state_dim+1] = 0.0 # y
			array[i*motion_state_dim+2] = 0.0 # theta
			array[i*motion_state_dim+3] = 0.0 # x'
			array[i*motion_state_dim+4] = 0.0 # y'
			array[i*motion_state_dim+5] = 0.0 # theta'
	return array

def qn():
	tf.reset_default_graph()
	print("Initialize tensorflow successfully!")
	
	## Define Q-network q(a,s) that ouput the rewards of 27 actions by given state.
	## The actions are discretized for simplity temporarily.
	## Other RL agents, such as DDPG and NAF, can be considered for continuous action space.
	inputs = tf.placeholder(shape=[1, traj_fixed_len * motion_state_dim], dtype=tf.float32)
	net = InputLayer(inputs, name='observation')
	net = DenseLayer(net, n_units=actions_num, act=tf.identity, name='q_a_s') # output layer
	outputs_gragh = net.outputs # action-value / rewards of 27 actions
	predict = tf.argmax(outputs_gragh, 1) # chose action greedily with reward. In Q-Learning, policy is greedy, so we use "max" to select the next action.
	print("Define Q-network q(a,s) successfully!")
	
	## Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
	nextQ = tf.placeholder(shape=[1, actions_num], dtype=tf.float32)
	loss = tl.cost.mean_squared_error(nextQ, outputs_gragh, is_mean = False) # tf.reduce_sum(tf.square(nextQ - y))
	train_op = tf.train.GradientDescentOptimizer(learning_rate = 0.1).minimize(loss)
	print("Define train operator successfully!")
	
	# parameter preset
	rospy.set_param("ramp/eval_weight_T", 1.75)
	rospy.set_param("ramp/eval_weight_D", 15.0)
	rospy.set_param("ramp/eval_weight_A", 0.01)
	
	# wait
	time.sleep(1) # 1.0s
	
	print("set e = 0.1")
	e = 0.1 # e-Greedy Exploration, the larger the more random
	rate = rospy.Rate(10) # 10hz
	do_nothing = decodeAction(13)
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
				last_start_flag = start_flag
				start_flag = rospy.get_param("ramp/start_planner")
				time.sleep(0.2) # 0.2s
				print("RL agent is waiting a new episode......")
			
			## start episode
			print("start episode!")
			i_episode += 1
			episode_time = time.time()
			s, _, d, _ = envStep(do_nothing) # do nothing, just get an initial observation, 13 -> ParameterUpdates(0, 0, 0, 0), note that there is bias
			while not s.feasible:
					s, _, d, _ = envStep(do_nothing)
					print("initial observation infeasible!")
			rAll = 0
			i_step = 0
			while True: # step
				i_step += 1
				## Choose an action by greedily (with e chance of random action) from the Q-network
				a, allQ = sess.run([predict, outputs_gragh], feed_dict={inputs : [buildQNetIn(s)]})
				## e-Greedy Exploration !!! sample random action
				if np.random.rand(1) < e:
					a[0] = math.floor(np.random.rand(1) * 27)
		        ## Get new state and reward from environment
				s1, r, d, _ = envStep(decodeAction(a[0]))
				## Obtain the Q' values by feeding the new state through our network
				Q1 = sess.run(outputs_gragh, feed_dict={inputs : [buildQNetIn(s1)]})
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
				_ = sess.run(train_op, {inputs : [buildQNetIn(s)], nextQ : targetQ})
				rAll += r
				s = s1
				print("done = ", d)
				if d == True:
					## Reduce chance of random action if an episode is done.
					e -= 0.01
					break
				rate.sleep()
				
	print("Exit qn()!")

if __name__ == "__main__":
	rospy.init_node(node_name, anonymous=True)
	try:
		# test()
		qn()
	except rospy.ROSInterruptException:
		pass
