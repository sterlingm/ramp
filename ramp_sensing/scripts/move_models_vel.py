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
    req.name = 'person_walking'
    req.relative_entity_name = 'link'

    # Call the service
    res = get_model_state(req.name, req.relative_entity_name)
    print res


    r = rospy.Duration(0.1)

    for x in range(0,10):
    
        res = get_model_state(req.name, req.relative_entity_name)

        # Set state
        setReq = ModelState()
        setReq.model_name = 'person_walking'
        setReq.reference_frame = 'map'
        setReq.pose.position.x = res.pose.position.x + 0.2
        setReq.pose.position.y = res.pose.position.y + 0.2

        # Call the service
        setRes = set_model_state(setReq)
        print setRes.success
    
        rospy.sleep(r)


    print "Exiting normally"



if __name__ == '__main__':
    main()
