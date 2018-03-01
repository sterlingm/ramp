#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Obstacle.h"
#include <tf/transform_datatypes.h>
#include <vector>
#include "utility.h"
#include "circle_packer.h"



class Obstacle 
{
  public:
    Obstacle();
    Obstacle(CircleGroup& cg);
    ~Obstacle(); 

    /** Data Members */
    ramp_msgs::Obstacle msg_;


    //Time of last update
    ros::Time last_updated_;

    // Transform
    tf::Transform T_w_init_;
    

    /** Methods */

    void update(const CircleGroup& c, const Velocity& v, const double theta);

    float radius_;
  private:

    Utility utility_;
};

#endif
