#!/home/kai/env/bin python3.4

import os
import sys
import rospy
from ramp_msgs.msg import RampObservationOneRunning

this_exe_info = None
rospy.init_node('test_ramp', anonymous = True)

def oneExeInfoCallback(data):
    global this_exe_info
    if this_exe_info is not None:
        ## TODO: log into file with date in its name
        print("Unused RampEnv.this_exe_info is overlaped!")
    this_exe_info = data

max_exe_time = 60.0
rospy.set_param("/max_exe_time", max_exe_time)
rospy.set_param("/ramp/eval_weight_T", 1.0)
rospy.set_param("/ramp/eval_weight_Qc", 1.0)
rospy.set_param("/ramp/eval_weight_Qk", 1.0)
one_exe_info_sub = rospy.Subscriber("ramp_collection_ramp_ob_one_run", RampObservationOneRunning,
		                            oneExeInfoCallback)
coes_for_test = [[0.0, 0.0],
                 [0.0, 1.0],
                 [1.0, 0.0],
                 [1.0, 1.0],
                 [0.0, 0.3],
                 [0.0, 0.5],
                 [0.3, 0.3],
                 [0.3, 0.5]]


step = 0
for i in range(len(coes_for_test)):
    rospy.set_param("/ramp/eval_weight_A", coes_for_test[i][0])
    rospy.set_param("/ramp/eval_weight_D", coes_for_test[i][1])
    for j in range(10):
        print("################################ STEP {} ################################".format(step))
        ## wait the actual environment to get ready......
        print("Wait the actual environment to get ready......")
        try:
            rospy.wait_for_service("env_ready_srv")
        except rospy.exceptions.ROSInterruptException:
            print("\nCtrl+C is pressed!")
            sys.exit()

        print("Set start_planner to true for to start one execution!")
        rospy.set_param("/ramp/start_planner", True)

        ## wait for this execution completes......
        check_exe_rate = rospy.Rate(20) # 20Hz
        has_waited_exe_for = 0 # seconds
        print("Wait for this execution completes......")
        start_waiting_time = rospy.get_rostime()
        while not rospy.core.is_shutdown() and this_exe_info is None:
            try:
                check_exe_rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                print("\nCtrl+C is pressed!")
                sys.exit()

            cur_time = rospy.get_rostime()
            has_waited_exe_for = cur_time.to_sec() - start_waiting_time.to_sec() # seconds
            if has_waited_exe_for >= max_exe_time + 20.0: # overtime
                print("Ramp_planner has been respawned from unexpected interruption, will set start_planner to true again.")
                print("Wait the actual environment to get ready......")

                try:
                    rospy.wait_for_service("env_ready_srv")
                except rospy.exceptions.ROSInterruptException:
                    print("\nCtrl+C is pressed!")
                    sys.exit()

                print("Set start_planner to true for to start one execution!")
                rospy.set_param("/ramp/start_planner", True)
                start_waiting_time = rospy.get_rostime()
                print("Wait for this execution completes......")

        if rospy.core.is_shutdown():
            sys.exit()

        print("A execution completes!")
        this_exe_info = None
        step += 1