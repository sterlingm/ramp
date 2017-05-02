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

Obstacle::Obstacle(int costmap_width, int costmap_height, float costmap_origin_x, float costmap_origin_y, float costmap_res) 
  : costmap_width_(costmap_width), costmap_height_(costmap_height), costmap_origin_x_(costmap_origin_x), costmap_origin_y_(costmap_origin_y), costmap_res_(costmap_res)
{
  float x_max = costmap_width_ + costmap_origin_x_;
  float x_min = x_max - costmap_width_;
  float y_max = costmap_height_ + costmap_origin_y_;
  float y_min = y_max - costmap_height_;

  x_translate_costmap_ = x_min;
  y_translate_costmap_ = y_min;
  ROS_INFO("width: %i height: %i o_x: %f o_y: %f x_max: %f x_min: %f y_max: %f y_min: %f", costmap_width_, costmap_height_, costmap_origin_x_, costmap_origin_y_, x_max, x_min, y_max, y_min);
}

Obstacle::~Obstacle() {}


void Obstacle::update(const nav_msgs::Odometry o) 
{
  /*ROS_INFO("Obstacle odometry passed in:\nPosition: (%f, %f, %f)", 
      o.pose.pose.position.x, 
      o.pose.pose.position.y, 
      tf::getYaw(o.pose.pose.orientation));*/

  //Set new odometry infomation
  odom_t      = o;

  doTF();

  //Update time
  last_updated_ = ros::Time::now();
}


void Obstacle::update(const Circle c, const Velocity& v, const double theta)
{
  cir_ = c;

  // Call doTF to update ms
  doTF(false);

  ramp_msgs::MotionState ms;
  ms.positions.push_back((c.center.x * costmap_res_) + x_translate_costmap_);
  ms.positions.push_back((c.center.y * costmap_res_) + y_translate_costmap_);
  ms.positions.push_back(theta);

  ms.velocities.push_back(v.vx);
  ms.velocities.push_back(v.vy);
  ms.velocities.push_back(0);

  msg_.ob_ms = ms;
  
  last_updated_ = ros::Time::now();
}

void Obstacle::doTF(bool odom)
{
  ramp_msgs::MotionState ms;
  ms.positions.clear();
  ms.velocities.clear();
  ms.accelerations.clear();

  tf::Vector3 p_st(odom_t.pose.pose.position.x, odom_t.pose.pose.position.y, 0); 
  
  /*
   * Comment out for system-level testing
   */
  if(odom)
  {
    tf::Vector3 p_st_tf = T_w_init_ * p_st;

    ms.positions.push_back(p_st_tf.getX());
    ms.positions.push_back(p_st_tf.getY());
     

    ms.positions.push_back(utility_.displaceAngle( tf::getYaw(T_w_init_.getRotation()), tf::getYaw(odom_t.pose.pose.orientation)));

    ms.velocities.push_back(odom_t.twist.twist.linear.x);
    ms.velocities.push_back(odom_t.twist.twist.linear.y);
    ms.velocities.push_back(odom_t.twist.twist.angular.z);

    std::vector<double> zero; zero.push_back(0); zero.push_back(0); 
    double theta  = utility_.findAngleFromAToB(zero, ms.positions);
    double phi    = ms.positions.at(2);
    double v      = ms.velocities.at(0);

    //////ROS_INFO("teta: %f phi: %f v: %f", teta, phi, v);


    /*
     * Comment out for system-level testing
     */
    ms.velocities.at(0) = v*cos(phi);
    ms.velocities.at(1) = v*sin(phi);


    /*
     * Comment out for system-level testing
     */
    if(v < 0) 
    {
      ms.positions.at(2) = utility_.displaceAngle(ms.positions.at(2), PI);
    }
  }

  // Transform based on costmap parameters
  else
  {
  }

  msg_.ob_ms = ms;
}

