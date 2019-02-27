#!/usr/bin/env python
import rospy
import numpy
import matplotlib.pyplot as plt
from ramp_msgs.msg import ObstacleList
from ramp_msgs.msg import Obstacle
    
pub_obs = rospy.Publisher('obstacles_lat_unc', ObstacleList, queue_size=10)


def obstaclesCb(data):
    print 'In obstaclesCb'
    obsOut = ObstacleList()

    # First just apply some latency
    applyLatency(ob)

    # For each obstacle, apply some uncertainty
    for ob in data.obstacles:
        #applyUncertainty(ob)
        obsOut.obstacles.append(ob)


    # Publish the obstacle list
    pub_obs.publish(obsOut)

def applyLatency(ob):
    print 'In applyLatency'

    rospy.sleep(0.25)
    


def applyUncertainty(ob):
    print 'In applyUncertainty'

    mean = [ob.ob_ms.positions[0], ob.ob_ms.positions[1]]
    cov = [ [0.001, 0.002], [0.002, 0.001] ]

    x, y = numpy.random.multivariate_normal(mean, cov, 5000).T
    plt.plot(x, y, 'x')
    plt.axis('equal')
    plt.show()

    


def main():
    rospy.init_node('ob_latency_uncertainty', anonymous=True)
    print 'In main'

    rospy.Subscriber('/obstacles', ObstacleList, obstaclesCb)

    test_ob = Obstacle()
    test_ob.ob_ms.positions.append(1)
    test_ob.ob_ms.positions.append(1)

    applyUncertainty(test_ob)


    rospy.spin()
    print 'Exiting normally'




if __name__ == '__main__':
    main()
