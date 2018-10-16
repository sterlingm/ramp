#!/usr/bin/env python
import rospy
import sys
import tf
import math
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyBodyWrenchRequest
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from ramp_msgs.msg import MotionState
from tf_conversions import transformations




class MyNode:

    def __init__(self):
        self.get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.num_obs = 2

        self.listener = tf.TransformListener()
        self.tf_pos = tuple()
        self.tf_rot = tuple()

        self.sub_update = rospy.Subscriber('update', MotionState, self.updateCb)

        self.robotPos = []



    def updateCb(self, msg):

        # Crease PoseStamped from MotionState msg
        m = PoseStamped()
        m.header.frame_id = '/odom'

        m.pose.position.x = msg.positions[0]
        m.pose.position.y = msg.positions[1]
        m.pose.orientation = Quaternion(*transformations.quaternion_from_euler(0,0,msg.positions[2]))

        mTF = self.listener.transformPose('/map', m)

        if len(self.robotPos) == 0:
            self.robotPos.append(mTF.pose.position.x)
            self.robotPos.append(mTF.pose.position.y)
        else:
            self.robotPos[0] = mTF.pose.position.x
            self.robotPos[1] = mTF.pose.position.y





    def start(self):
        # Get the transform
        rospy.sleep(1)
        (self.tf_pos, self.tf_rot) = self.listener.lookupTransform('map', 'odom', rospy.Time(0))

        r = rospy.Duration(0.05)

        inc = 0.1
        while True:
                for x in range(0, 5):
                    for i in range(0, self.num_obs):
                        
                        # Moving obstacle i
                        model_name = 'cardboard_box_%d' % i

                        req = GetModelState()
                        req.name = model_name
                        req.relative_entity_name = 'link'

                        # Call the service
                        res = self.get_model_state(req.name, req.relative_entity_name)
                        

                        # Get distance to robot
                        distToRobot = math.sqrt( math.pow(self.robotPos[0]-res.pose.position.x, 2) + math.pow(self.robotPos[1]-res.pose.position.y, 2) )
                        print 'distToRobot: %s' % distToRobot

                        # cardboard box radius is about .25m, turtlebot radius is also about .2m
                        # Set it higher to account for some network lag
                        if distToRobot > 1.0:
                            # Set new state
                            setReq = ModelState()
                            setReq.model_name = model_name
                            setReq.reference_frame = 'map'

                            if i % 2 == 0:
                                setReq.pose.position.x = res.pose.position.x + inc
                                setReq.pose.position.y = res.pose.position.y + inc
                            else:
                                setReq.pose.position.x = res.pose.position.x - inc
                                setReq.pose.position.y = res.pose.position.y - inc

                            #setReq.pose.position.y = res.pose.position.y

                            # Call the service
                            setRes = self.set_model_state(setReq)
                        
                        rospy.sleep(r)
                inc *= -1




def main():
    rospy.init_node("move_models_control", anonymous=False)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N


    n = MyNode()
    n.start()



    print "Exiting normally"



if __name__ == '__main__':
    main()
