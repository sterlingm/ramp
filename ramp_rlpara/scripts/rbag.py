#!/usr/bin/env python3.4

import rosbag
bag = rosbag.Bag('t4.bag')

for topic, msg, t in bag.read_messages(topics=['/chatter', '/numbers']):
    print(topic, msg, t)
    
    #if topic.decode() == 'chatter':
    #	print("process chatter: " + msg.data)
    	
    #if topic.decode() == 'numbers':
    #	print("process numbers: " + str(msg.data))

print("end")
bag.close()

# roslib rosbag genmsg genpy genmsg != str
