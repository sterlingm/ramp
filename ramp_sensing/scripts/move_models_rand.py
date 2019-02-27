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

        #speeds = [0.2, 0.2]
        speeds = [0.33, 0.33, 1.0, 1.0]
        ds = [s * r.to_sec() for s in speeds]
        print ds

        directions = [(random.randrange(-180, 180) / 180.0) * math.pi for i in range(0,self.num_obs)]

        ob_x_range = [0.0, 1.5]
        ob_y_range = [2.5, 4.5]

        ob_fast_x_range = [2.5, 4.5]
        ob_fast_y_range = [0.5, 2.0]
 
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

                prevObPos[i] = [res.pose.position.x, res.pose.position.y]

                # Get min dist to other obstacles
                i_ob = 0 if i == 1 else 1
                minDistToOtherOb = 10
                i_minDistOb = 0
                for i_ob in range(0,self.num_obs):
                    if i != i_ob:
                        dist = math.sqrt( math.pow(prevObPos[i_ob][0]-res.pose.position.x, 2) + math.pow(prevObPos[i_ob][1]-res.pose.position.y, 2) )
                        if dist < minDistToOtherOb:
                            minDistToOtherOb = dist;
                            i_minDistOb = i_ob
                    

                print 'distToRobot: %s minDistToOtherOb: %s' % (distToRobot, minDistToOtherOb)

                # cardboard box radius is about .25m, turtlebot radius is also about .2m
                # Set it higher to account for some network lag
                if distToRobot > 1.0:


                    # If the obstacles are too close then change direction
                    # We check this later on
                    #if distToOtherOb < 0.6:
                    #    directions[i] = -directions[i]



                    # Set new state
                    setReq = ModelState()
                    setReq.model_name = model_name
                    setReq.reference_frame = 'map'

                    # Random distance and direction
                    #dist = random.random() / 50.0
                    #direction = (random.randrange(-180, 180) / 180.0) * math.pi
                    #deltax = dist*math.cos(direction)
                    #deltay = dist*math.sin(direction)

                    # Corresponds to all obstacles

                    # Corresponding changes in x and y
                    deltax = ds[i]*math.cos(directions[i])
                    deltay = ds[i]*math.sin(directions[i])
                    print 'dist: %s direction: %s deltax: %s deltay: %s' % (ds[i], directions[i], deltax, deltay)

                    # Set new position
                    setReq.pose.position.x = res.pose.position.x + deltax
                    setReq.pose.position.y = res.pose.position.y + deltay
                    x = setReq.pose.position.x
                    y = setReq.pose.position.y

                    x_min = ob_x_range[0] if i < 2 else ob_fast_x_range[0]
                    x_max = ob_x_range[1] if i < 2 else ob_fast_x_range[1]
                    y_min = ob_y_range[0] if i < 2 else ob_fast_y_range[0]
                    y_max = ob_y_range[1] if i < 2 else ob_fast_y_range[1]

                    # Specific to certain obstacles

                    # Reset the direction if we are out of bounds
                    if x >= x_max or x <= x_min or y >= y_max or y <= y_min or minDistToOtherOb < 0.6:

                        #if minDistToOtherOb < 0.6:
                        #    if directions[i] 

                        print 'x: %s y: %s dist: %s direction: %s deltax: %s deltay: %s' % (x, y, ds[i], directions[i], deltax, deltay)
                        if x >= x_max:
                            directions[i] = (random.randrange(90,180) / 180.0) * math.pi
                        elif x <= x_min:
                            directions[i] = (random.randrange(0,90) / 180.0) * math.pi
                        elif y >= y_max:
                            directions[i] = (random.randrange(-180,0) / 180.0) * math.pi
                        elif y <= y_min:
                            directions[i] = (random.randrange(0, 180) / 180.0) * math.pi
                        elif minDistToOtherOb < 0.6:
                            relative_dir = math.atan2(prevObPos[i_minDistOb][0] - x, prevObPos[i_minDistOb][1] - y)
                            directions[i] = displaceAngle(relative_dir, math.pi)
                            print 'relative_dir: %s directions[%s]: %s' % (relative_dir, i, directions[i])
                        
                        # Speed the ob up to escape getting stuck in a corner
                        # Add .001
                        deltax = (ds[i]+0.001)*math.cos(directions[i])
                        deltay = (ds[i]+0.001)*math.sin(directions[i])

                        # Set new position
                        setReq.pose.position.x = res.pose.position.x + deltax
                        setReq.pose.position.y = res.pose.position.y + deltay

                    # Call the service
                    setRes = self.set_model_state(setReq)

                 
                rospy.sleep(r)




def findDistanceBetweenAngles(a, b):
  difference = b - a
  result = 0
  
  # If difference > pi, the result should be in [-PI,0] range
  if(difference > math.pi):
    difference = math.fmod(difference, math.pi)
    result = difference - math.pi
  

  # If difference < -pi, the result should be in [0,PI] range
  elif(difference < -math.pi):
    result = difference + (2.0*math.pi)

  # Else, the difference is fine
  else:
    result = difference

  return result


def displaceAngle(angle, amount):
  amount = math.fmod(amount, 2.0*math.pi)

  if(amount > math.pi):
    amount = math.fmod(amount,math.pi) - math.pi

  return findDistanceBetweenAngles(-angle, amount)



def main():
    rospy.init_node("move_models_rand", anonymous=False)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N


    n = MyNode()
    n.start()



    print "Exiting normally"



if __name__ == '__main__':
    main()
