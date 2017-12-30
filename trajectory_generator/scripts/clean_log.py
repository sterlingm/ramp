#!/usr/bin/env python

import os
## maybe this node will be respawn again and again
os.system("rm -rf /home/kai/.ros/log")
print("/home/kai/.ros/log is cleaned!")

import rospy
import time

rospy.init_node('clean_log', anonymous = True)

sleep_time = 10.0 # seconds
cnt = 0
while not rospy.core.is_shutdown():
	time.sleep(sleep_time)
	cnt += 1
	print(str(cnt * sleep_time) + "s has elapsed since last cleaning...")
	
	if cnt >= 6*3: # 3 minutes
		cnt = 0
		os.system("rm -rf /home/kai/.ros/log")
		print("/home/kai/.ros/log is cleaned!")
