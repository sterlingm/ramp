#!/usr/bin/env python
import rospy
import sys
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyBodyWrenchRequest
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def main():
    rospy.init_node("move_gazebo_object_vel", anonymous=False)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print 'N: %d' % N

    # Get list of models?

    # Initialize the service clients
    get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    req = GetModelState()
    req.name = 'cardboard_box'
    req.relative_entity_name = 'link'

    # Call the service
    res = get_model_state(req.name, req.relative_entity_name)
    print res


    r = rospy.Duration(0.1)

    inc = 0.1
    while True:
        for x in range(0,10):
        
            res = get_model_state(req.name, req.relative_entity_name)

            # Set state
            setReq = ModelState()
            setReq.model_name = 'cardboard_box'
            setReq.reference_frame = 'map'
            setReq.pose.position.x = res.pose.position.x
            setReq.pose.position.y = res.pose.position.y + inc

            # Call the service
            setRes = set_model_state(setReq)
            print setRes.success
        
            rospy.sleep(r)
        inc *= -1


    print "Exiting normally"



if __name__ == '__main__':
    main()
