#!/usr/bin/env python

import math
import rospy
from ramp_msgs.srv import *
from ramp_msgs.msg import ParameterUpdates
from ramp_msgs.msg import RampTrajectory

env_service_name = 'response2para_update'
node_name = 'agent_dqn'
goal_radius = 1.0 #  TODO unit: ?

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

def envStep(act):
	rospy.wait_for_service(env_service_name)
	try:
		getBestTrajectory = rospy.ServiceProxy(env_service_name, EnvironmentSrv)
		response = getBestTrajectory(act)
		if response.is_valid == True:
			best_trajectory = response.trajectory
			len_best_trajectory = getTrajectoryLength(best_trajectory) #  TODO unit: ?
			reward = -len_best_trajectory
			if len_best_trajectory < goal_radius:
				done = True
			else:
				done = False
			is_success = True
		else:
			best_trajectory = RampTrajectory()
			len_best_trajectory = getTrajectoryLength(best_trajectory) # TODO unit: ?
			reward = -len_best_trajectory
			done = False
			is_success = False
		return best_trajectory, reward, done, is_success
	except rospy.ServiceException as exc:
		print("EnvironmentSrv did not process request: " + str(exc))

# def dqn(): # TODO or some other RL agent

if __name__ == "__main__":
	rospy.init_node(node_name, anonymous=True)
	try:
		test()
	except rospy.ROSInterruptException:
		pass
