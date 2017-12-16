#!/usr/bin/env python

import rosbag

bag = rosbag.Bag('/home/kai/data/test/test_read_BZ2.bag')
cnt = 0
for topic, msg, t in bag.read_messages(topics=['/bestTrajec']):
    cnt += 1
    for i in range(0, len(msg.trajectory.points)):
        print(msg.trajectory.points[i].accelerations)
    if cnt > 15:
        break
bag.close()
