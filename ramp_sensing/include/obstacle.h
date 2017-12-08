#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Obstacle.h"
#include "ramp_msgs/PackedObstacle.h"
#include <tf/transform_datatypes.h>
#include <vector>
#include "utility.h"
#include "circle_packer.h"



class Obstacle 
{
  public:
    Obstacle();
    Obstacle(const nav_msgs::Odometry o);
    Obstacle(float radius, int costmap_width, int costmap_height, float costmap_origin_x, float costmap_origin_y, float costmap_res, float global_grid_origin_x, float global_grid_origin_y);
    ~Obstacle(); 

    /** Data Members */
    ramp_msgs::Obstacle msg_;

    // Hold odometry information for t and t-1
    // Leave this here for system-level experiments
    nav_msgs::Odometry odom_t;

    //Time of last update
    ros::Time last_updated_;

    // Transform
    tf::Transform T_w_init_;
    

    /** Methods */

    void update(const nav_msgs::Odometry o);

    void update(const CircleGroup& c, const Velocity& v, const double theta);

  private:
    int costmap_width_;
    int costmap_height_;
    float costmap_origin_x_;
    float costmap_origin_y_;
    float costmap_res_;
    float x_translate_costmap_;
    float y_translate_costmap_;
    float global_grid_origin_x_;
    float global_grid_origin_y_;
    float radius_;

    Utility utility_;
};

#endif
