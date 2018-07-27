#!/usr/bin/env python
import rospy
import sys
import tf
import random
import math
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyBodyWrenchRequest
from gazebo_msgs.srv import BodyRequest
from gazebo_msgs.srv import BodyRequestRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Point


def checkSpacing(pos, existing_pos):
    for i in range(len(existing_pos)):
        d = math.sqrt(math.pow( pos.x - existing_pos[i].x, 2) + math.pow(pos.y
            - existing_pos[i].y,2) )
        if d < 1:
            return False
    return True



def main():
    rospy.init_node("move_gazebo_object", anonymous=False)
    print 'Script: %s' % sys.argv[0]

    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N

    x_min = -7.5
    x_max = 6
    y_min = -1.25
    y_max = 0.75


# Create SpawnModelRequest object, set member values that won't change
    req = SpawnModelRequest()

    #f = open('/home/sterlingm/.gazebo/models/person_walking/model.sdf', 'r')
    f = open('/home/sterlingm/.gazebo/models/cardboard_box/model.sdf', 'r')
    sdff = f.read()
    req.model_xml = sdff

    req.reference_frame = 'world'

    positions = []

    # Spawn N walking person models
    for i in range(0,N):

        # Create the model name
        s = 'cardboard_box_%d' % i if i > 0 else 'cardboard_box'
        print 's: %s' % s
        req.model_name = s

        # Create the Point object for its position
        p = Point()
        
        # Emulate a do-while loop
        count = 0
        while True:
            # Get random position
            p.x = random.uniform(x_min, x_max)
            p.y = random.uniform(y_min, y_max)
            p.z = 0.1
            
            # Check if it is spaced out enough (stop trying after so many 
            # attempts)
            if checkSpacing(p, positions) or count > 1000:
                break
            count = count + 1
        
        # Set model pose
        req.initial_pose.position = p

        # Set orientation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.5708)
        req.initial_pose.orientation.x = quaternion[0]
        req.initial_pose.orientation.y = quaternion[1]
        req.initial_pose.orientation.z = quaternion[2]
        req.initial_pose.orientation.w = quaternion[3]

        # Store the object's position
        positions.append(p)

        # Call the service to spawn the model
        spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model(req)
        print 'Spawn response: ', resp.success
        print 'Spawn status_message: %s' % resp.status_message
    

    print "Exiting normally"



if __name__ == '__main__':
    main()
