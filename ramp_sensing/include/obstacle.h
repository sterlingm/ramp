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
<<<<<<< HEAD
    int costmap_width_;
    int costmap_height_;
    float costmap_origin_x_;
    float costmap_origin_y_;
    float costmap_res_;
    float x_translate_costmap_;
    float y_translate_costmap_;
    float global_grid_origin_x_;
    float global_grid_origin_y_;
=======
>>>>>>> devel

    Utility utility_;
};

#endif
