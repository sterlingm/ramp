#!/usr/bin/env python
import rospy
import os
import rospkg
import sys
import time
from nav_msgs.msg import OccupancyGrid
from ramp_msgs.msg import HilbertMap

# hilbert_map.csv gets written in my_main_SBHM.py
def main():
    rospy.init_node('hmap', anonymous=False)

    print 'sys:'
    if len(sys.argv) > 2:
        for s in sys.argv:
            print s

    
    # Setup paths
    rospack = rospkg.RosPack()
    ros_pkg_path = rospack.get_path('ramp_sensing')

    # If a filename was passed in then set it to that, otherwise set to hilbert map filename
    fname = sys.argv[1] if len(sys.argv) > 1 else 'hilbert_map.csv'
    p = os.path.join(ros_pkg_path, fname)
    print p

    f = open(p, 'r')
    lines = f.read().split('\n')

    grid = OccupancyGrid()

    xs = []
    ys = []

    gamma = 0

    # For each line
    for i,l in enumerate(lines):
        #print l

        # Get values for the line
        nums = l.split(',')

        if len(nums) > 1:

            # Get x and y
            xs.append(float(nums[0]))
            ys.append(float(nums[1]))

            # Append the probability value as an int
            p = float(nums[2])
            if p < 1 and p > 0:
                p = p * 100

            grid.data.append(int(p))

            if len(nums) > 3:
                if gamma == 0:
                    gamma = float(nums[3])
                    #print('gamma: %f' % gamma)

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    print('x_min: %f x_max: %f y_min: %f y_max: %f' % (x_min, x_max, y_min, y_max))
    
    # Set other properties of grid
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    # Added +1 for static map for medium env 1
    if len(sys.argv) > 1:
        grid.info.width = ((x_max - x_min + grid.info.resolution) * 20)
        grid.info.height = ((y_max - y_min + grid.info.resolution) * 20)
    else:
        grid.info.width = ((x_max - x_min + grid.info.resolution) * 20)
        grid.info.height = ((y_max - y_min + grid.info.resolution) * 20)

    #print('(x_max - x_min - grid.info.resolution): %f' % \
            #(x_max - x_min + grid.info.resolution))
    
    grid.info.origin.position.x = xs[0]
    grid.info.origin.position.y = ys[0]
    #grid.info.width = len(lines) * grid.info.resolution
    #grid.info.height = len(lines

    # Set topic name for occupancy grid
    gridTopicName = sys.argv[2] if len(sys.argv) > 2 else 'hilbert_map_grid'

    # Create Publisher 
    pub = rospy.Publisher('hilbert_map', HilbertMap, queue_size=1)
    pubRviz = rospy.Publisher(gridTopicName, OccupancyGrid, queue_size=1)

    hmap = HilbertMap()
    hmap.map = grid
    hmap.gamma = gamma

    #sys.stdin.flush()
    #sys.stdout.flush()
    #v = raw_input("Press Enter to publish grid")

    time.sleep(1)

    if len(sys.argv) < 2:
        pub.publish(hmap)
    pubRviz.publish(hmap.map)


    print('\nExiting normally\n')


if __name__ == '__main__':
    main()
