#include "obstacle.h"

Obstacle::Obstacle() {
  nav_msgs::Odometry temp;
  temp.pose.pose.orientation.w = 1.;
  odom_t      = temp;
}

Obstacle::Obstacle(const nav_msgs::Odometry o) 
{
  odom_t      = o;
}

Obstacle::Obstacle(float radius, int costmap_width, int costmap_height, float costmap_origin_x, float costmap_origin_y, float costmap_res, float global_grid_origin_x, float global_grid_origin_y) 
  : costmap_width_(costmap_width), costmap_height_(costmap_height), costmap_origin_x_(costmap_origin_x), costmap_origin_y_(costmap_origin_y), costmap_res_(costmap_res)
{
  radius_ = radius;
  msg_.radius = radius;

  float x_max = costmap_width_ + costmap_origin_x_;
  float x_min = x_max - costmap_width_;
  float y_max = costmap_height_ + costmap_origin_y_;
  float y_min = y_max - costmap_height_;
    
  global_grid_origin_x_ = global_grid_origin_x;
  global_grid_origin_y_ = global_grid_origin_y;

  x_translate_costmap_ = x_min;
  y_translate_costmap_ = y_min;
  //ROS_INFO("width: %i height: %i o_x: %f o_y: %f x_max: %f x_min: %f y_max: %f y_min: %f", costmap_width_, costmap_height_, costmap_origin_x_, costmap_origin_y_, x_max, x_min, y_max, y_min);
}

Obstacle::~Obstacle() {}


void Obstacle::update(const nav_msgs::Odometry o)
{
  /*//ROS_INFO("Obstacle odometry passed in:\nPosition: (%f, %f, %f)", 
      o.pose.pose.position.x, 
      o.pose.pose.position.y, 
      tf::getYaw(o.pose.pose.orientation));*/

  //Set new odometry infomation
  odom_t      = o;

  //Update time
  last_updated_ = ros::Time::now();
}


void Obstacle::update(const CircleGroup& c, const Velocity& v, const double theta)
{
  /*
   *  Set the new CircleGroup
   */
  msg_.cirGroup.fitCir.center.x = c.fitCir.center.x;
  msg_.cirGroup.fitCir.center.y = c.fitCir.center.y;
  msg_.cirGroup.fitCir.radius = c.fitCir.radius;
  for(int i=0;i<c.packedCirs.size();i++)
  {
    ramp_msgs::Circle pc;
    pc.center.x = c.packedCirs[i].center.x;
    pc.center.y = c.packedCirs[i].center.y;
    pc.radius   = c.packedCirs[i].radius;

    msg_.cirGroup.packedCirs.push_back(pc);
  }

  // Update motion state
  msg_.ob_ms.positions.clear();
  msg_.ob_ms.velocities.clear();

  msg_.ob_ms.positions.push_back(c.fitCir.center.x);
  msg_.ob_ms.positions.push_back(c.fitCir.center.y);
  msg_.ob_ms.positions.push_back(theta);

  msg_.ob_ms.velocities.push_back(v.vx);
  msg_.ob_ms.velocities.push_back(v.vy);
  msg_.ob_ms.velocities.push_back(0);
  
  last_updated_ = ros::Time::now();
}

