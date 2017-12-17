#!/usr/bin/env python

'''
This file represents the actual environment in simulation. Once this actual environment is run, a whole learning is ready to start
(Wait the RL agent start the learning). The actual environment can also be the .bag files in the harddisk, which log the
data when running this file (the true actual environment in simulation).
'''

import os
import datetime
import signal
import subprocess
from subprocess import check_output

## make directory of this data collection, using current date
home_dir = os.getenv("HOME")
cur_date = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
file_dir = home_dir + '/data/ramp/simulation_env_data/' + cur_date + '/raw_data/'
os.system('mkdir -p ' + file_dir)

global_loop_cnt = 0
while True: # TODO: enable key interrupt or some other
    global_loop_cnt += 1

    ## one loop one .bag file, decide the file name here
    #  file_name: globalloopcnt.bag
    ## TODO: transfer .bag files' names as follows in another script
    #  structure: loop - step - episode
    #  file name: localloopcnt_localstepcnt_episodecnt.bag
    file_name = str(global_loop_cnt)

    ## record differet topics, using BZ2 compression
    #  use regular expression to include or exclude topics
    #  topics with name "/ramp_collection_.*" will all be recorded
    #  here add the specific topic you want to record,
    #  such as ..., "/ramp_collection_.*", "/topic_name_1", "/topic_name_2", ..., "/topic_name_n"
    #  create a child process and return the object of it (rosbag_proc)
    rosbag_proc = subprocess.Popen(["rosbag", "record", "--bz2", "--output-name=" + file_dir + file_name, "--regex", "/ramp_collection_.*", "/bestTrajec"])

    ## reset the robot in Gazebo to the start point
    os.system('rosservice call /gazebo/reset_world "{}"')

    ## start ramp_planner and other necessary nodes and waiting for the /ramp/start_planner to be True
    os.system('roslaunch ramp_launch planner_full_costmap_simulation_qn.launch')

    ## kiil (interrupt) the child rosbag process here
    rosbag_proc_name = "/opt/ros/indigo/lib/rosbag/record"
    try:
        pids = check_output(["pidof", rosbag_proc_name])
    except subprocess.CalledProcessError:
        # check_output failed
        print("Call check_output failed. Maybe process with name " + rosbag_proc_name + " doesn't exist.")
    else:
        # check_output successed
        pids = pids.decode()
        pids = pids.split()
        for i in range(len(pids)):
            pid = int(pids[i])
            try: # do not let this process end using try
                os.kill(pid, signal.SIGINT)
            except OSError:
                # pid don't exist
                print("pid " + str(pid) + " don't exist!")
            else:
                # kill pid success
                print("pid " + str(pid) + " has died!")

    ## end of one running of the robot (note that one step of learning may contain many runnings)
    print("=================================================== end ===================================================")