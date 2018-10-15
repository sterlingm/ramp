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

    num_obs = 1

    r = rospy.Duration(0.1)

    inc = 0.1
    while True:

            for x in range(0,10):
                for i in range(0,num_obs):

                    # Moving obstacle i
                    model_name = 'cardboard_box_%d' % i

                    req = GetModelState()
                    req.name = model_name
                    req.relative_entity_name = 'link'

                    # Call the service
                    res = get_model_state(req.name, req.relative_entity_name)
                    print res

                    # Set new state
                    setReq = ModelState()
                    setReq.model_name = model_name
                    setReq.reference_frame = 'map'

                    if i % 2 == 0:
                        setReq.pose.position.x = res.pose.position.x + inc
                        #setReq.pose.position.y = res.pose.position.y + inc
                    else:
                        setReq.pose.position.x = res.pose.position.x - inc
                        #setReq.pose.position.y = res.pose.position.y - inc

                    setReq.pose.position.y = res.pose.position.y

                    # Call the service
                    setRes = set_model_state(setReq)
                    print setRes.success
                
                    rospy.sleep(r)
            inc *= -1


    print "Exiting normally"



if __name__ == '__main__':
    main()
