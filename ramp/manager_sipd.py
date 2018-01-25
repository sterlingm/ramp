import time
import rospy
import signal
import subprocess
from subprocess import check_output
from ramp_msgs.msg import RampObservationOneRunning

watch_dog_cnt = 0

def oneExeInfoCallback(data):
    global watch_dog_cnt
    watch_dog_cnt = 0

rospy.init_node('manager')
exe_info_suber = rospy.Subscriber("ramp_collection_ramp_ob_one_run", RampObservationOneRunning,
                                oneExeInfoCallback)

while not rospy.core.is_shutdown():
    rviz_proc = subprocess.Popen(['roslaunch', 'ramp_launch', 'my_view_robot_costmap.launch'])
    time.sleep(3.0)
    gaze_proc = subprocess.Popen(['roslaunch', 'ramp_launch', 'gazebo_costmap_sipd.launch'])
    time.sleep(3.0)
    clean_proc = subprocess.Popen(['roslaunch', 'trajectory_generator', 'clean_log.launch'])
    time.sleep(3.0)
    plan_proc = subprocess.Popen(['roslaunch', 'ramp_launch', 'planner_full_costmap_simulation_sipd.launch'])
    time.sleep(3.0)

    while not rospy.core.is_shutdown():
        time.sleep(1.0)
        watch_dog_cnt += 1
        if watch_dog_cnt > 200:
            print('Restart!')
            watch_dog_cnt = 0
            break

    plan_proc.terminate()
    time.sleep(3.0)
    clean_proc.terminate()
    time.sleep(3.0)
    gaze_proc.terminate()
    time.sleep(3.0)
    rviz_proc.terminate()
    time.sleep(3.0)