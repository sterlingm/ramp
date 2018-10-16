#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)

def costmapCb(data):
    print 'In costmapCb'
    pub.publish(data)



def main():
    rospy.init_node('pub_costmap_as_map', anonymous=False)

    #rospy.Subscriber('costmap_node/costmap/costmap', OccupancyGrid, costmapCb)
    rospy.Subscriber('global_costmap', OccupancyGrid, costmapCb)

    rospy.spin()



if __name__ == '__main__':
    main()
    print 'Exiting normally'
