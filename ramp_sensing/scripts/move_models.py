#!/usr/bin/env python
import rospy
import sys
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyBodyWrenchRequest
from gazebo_msgs.srv import BodyRequest
from gazebo_msgs.srv import BodyRequestRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from geometry_msgs.msg import Wrench

#/gazebo/spawn_sdf_model


def main():
    rospy.init_node("move_gazebo_object", anonymous=False)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N

    # Get list of models?

    # Initialize the service clients
    clear_body_wrenches = rospy.ServiceProxy('gazebo/clear_body_wrenches', 
            BodyRequest)
    apply_body_wrench = rospy.ServiceProxy('gazebo/apply_body_wrench', 
            ApplyBodyWrench)
    
    # Create BodyRequestRequest to clear wrenches
    # Create ApplyBodyWrenchRequest to apply a wrench
    body_req = BodyRequestRequest()
    apply_req = ApplyBodyWrenchRequest()
    
    # Set the member variables that won't change
    apply_req.start_time = rospy.Time.now()
    apply_req.duration = rospy.Duration(2)

    # Create Wrench object
    wrench = Wrench()

    # First, clear any wrenches being applied
    for i in range(0,N):
        
        # Create the model and body name
        s = 'person_walking_%d::link' % i if i > 0 else 'person_walking::link'
        print 's: %s' % s
        body_req.body_name = s

        # Call clear_body_wrenches service
        clear_body_wrenches(body_req)

        # Setup ApplyBodyWrenchRequest object
        #apply_req.model_name = s
        apply_req.body_name = s

        # Check if 'world' works better for this
        apply_req.reference_frame = s
    
        # Make the wrench
        wrench.force.x = 10
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.z = 0
        apply_req.wrench = wrench


        # Call service to apply the wrench
        resp = apply_body_wrench(apply_req)



    print "Exiting normally"



if __name__ == '__main__':
    main()
