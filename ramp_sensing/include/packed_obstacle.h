#ifndef PACKED_OBSTACLE_H
#define PACKED_OBSTACLE_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/PackedObstacle.h"
#include <tf/transform_datatypes.h>
#include <vector>
#include "utility.h"
#include "circle_packer.h"



class PackedObstacle 
{
  public:
    PackedObstacle();
    PackedObstacle(const std::vector<Circle> cirs);
    ~PackedObstacle(); 

    /** Data Members */
    ramp_msgs::PackedObstacle msg_;

    //Time of last update
    ros::Time last_updated_;

    /** Methods */
  private:
    Utility utility_;
};

#endif
