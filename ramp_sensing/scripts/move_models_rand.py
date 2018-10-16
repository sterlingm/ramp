#!/usr/bin/env python
import rospy
import sys
import tf
import math
import random
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
        self.num_obs = 1

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

        r = rospy.Duration(0.01)

        random.seed(rospy.Time.now())

        speeds = [0.33, 0.33]
        ds = [s * r.to_sec() for s in speeds]
        print ds

        directions = [(random.randrange(-180, 180) / 180.0) * math.pi for i in range(0,self.num_obs)]

        ob_x_range = [0.25, 2.25]
        ob_y_range = [2.75, 4.0]

        prevObPos = [[0,0] for i in range(0,self.num_obs)] 

        while True:
            for i in range(0, self.num_obs):
                
                # Moving obstacle i
                model_name = 'cardboard_box_%d' % i
                print model_name

                req = GetModelState()
                req.name = model_name
                req.relative_entity_name = 'link'

                # Call the service
                res = self.get_model_state(req.name, req.relative_entity_name)
                

                # Get distance to robot
                distToRobot = math.sqrt( math.pow(self.robotPos[0]-res.pose.position.x, 2) + 
                        math.pow(self.robotPos[1]-res.pose.position.y, 2) )

                distToOtherOb = 0
                prevObPos[i] = [res.pose.position.x, res.pose.position.y]

                # Get min dist to other obstacles
                i_ob = 0 if i == 1 else 1
                distToOtherOb = 10
                if self.num_obs > 1:
                    distToOtherOb = math.sqrt( math.pow(prevObPos[i_ob][0]-res.pose.position.x, 2) + math.pow(prevObPos[i_ob][1]-res.pose.position.y, 2) )    
                    

                print 'distToRobot: %s distToOtherOb: %s' % (distToRobot, distToOtherOb)

                # cardboard box radius is about .25m, turtlebot radius is also about .2m
                # Set it higher to account for some network lag
                if distToRobot > 1.0:


                    # If the obstacles are too close then change direction
                    if distToOtherOb < 0.6:
                        directions[i] = -directions[i]



                    # Set new state
                    setReq = ModelState()
                    setReq.model_name = model_name
                    setReq.reference_frame = 'map'

                    # Random distance and direction
                    #dist = random.random() / 50.0
                    #direction = (random.randrange(-180, 180) / 180.0) * math.pi
                    #deltax = dist*math.cos(direction)
                    #deltay = dist*math.sin(direction)

                    # Corresponding changes in x and y
                    deltax = ds[i]*math.cos(directions[i])
                    deltay = ds[i]*math.sin(directions[i])
                    print 'dist: %s direction: %s deltax: %s deltay: %s' % (ds[i], directions[i], deltax, deltay)

                    # Set new position
                    setReq.pose.position.x = res.pose.position.x + deltax
                    setReq.pose.position.y = res.pose.position.y + deltay
                    x = setReq.pose.position.x
                    y = setReq.pose.position.y

                    if x >= ob_x_range[1] or x <= ob_x_range[0] or y >= ob_y_range[1] or y <= ob_y_range[0] or distToOtherOb < 0.6:
                        # Speed the ob up to escape getting stuck in a corner
                        deltax = (ds[i]+0.01)*math.cos(directions[i])
                        deltay = (ds[i]+0.01)*math.sin(directions[i])
                        print 'dist: %s direction: %s deltax: %s deltay: %s' % (ds[i], directions[i], deltax, deltay)
                        if x >= ob_x_range[1]:
                            directions[i] = (random.randrange(90,180) / 180.0) * math.pi
                        elif x <= ob_x_range[0]:
                            directions[i] = (random.randrange(0,90) / 180.0) * math.pi
                        elif y >= ob_y_range[1]:
                            directions[i] = (random.randrange(-180,0) / 180.0) * math.pi
                        elif y <= ob_y_range[0]:
                            directions[i] = (random.randrange(0, 180) / 180.0) * math.pi

                        # Set new position
                        setReq.pose.position.x = res.pose.position.x + deltax
                        setReq.pose.position.y = res.pose.position.y + deltay

                    # Call the service
                    setRes = self.set_model_state(setReq)
                
                rospy.sleep(r)




def main():
    rospy.init_node("move_models_control", anonymous=False)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N


    n = MyNode()
    n.start()



    print "Exiting normally"



if __name__ == '__main__':
    main()
