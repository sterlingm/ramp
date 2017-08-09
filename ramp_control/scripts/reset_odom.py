#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

def main():
    rospy.init_node("reset_odom", anonymous=False)

    pub = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, 
            queue_size=10)

    e = Empty()

    r = rospy.Rate(1)
    r.sleep()
    
    pub.publish(e)



if __name__ == '__main__':
    main()

