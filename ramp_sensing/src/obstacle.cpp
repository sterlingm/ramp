#include "obstacle.h"

Obstacle::Obstacle() {}

Obstacle::Obstacle(CircleGroup& cg)
{
  msg_.cirGroup.fitCir.center.x = cg.fitCir.center.x;
  msg_.cirGroup.fitCir.center.y = cg.fitCir.center.y;
  msg_.cirGroup.fitCir.radius = cg.fitCir.radius;
  
  for(int i=0;i<cg.packedCirs.size();i++)
  {
    ramp_msgs::Circle pc;
    pc.center.x = cg.packedCirs[i].center.x;
    pc.center.y = cg.packedCirs[i].center.y;
    pc.radius   = cg.packedCirs[i].radius;

    msg_.cirGroup.packedCirs.push_back(pc);
  }
  
  // Update motion state
  msg_.ob_ms.positions.clear();
  msg_.ob_ms.velocities.clear();

  msg_.ob_ms.positions.push_back(cg.fitCir.center.x);
  msg_.ob_ms.positions.push_back(cg.fitCir.center.y);
  msg_.ob_ms.positions.push_back(0);

  msg_.ob_ms.velocities.push_back(0);
  msg_.ob_ms.velocities.push_back(0);
  msg_.ob_ms.velocities.push_back(0);
  
  last_updated_ = ros::Time::now();
}

Obstacle::~Obstacle() {}

// Tf is needed since update only gives odom data
void Obstacle::update(const nav_msgs::Odometry& o)
{
  //ROS_INFO("In Obstacle::update(Odometry)");
  //ROS_INFO("T_w_init_.p: (%f,%f,%f) z_rotation: %f", T_w_init_.getOrigin().getX(), T_w_init_.getOrigin().getY(), T_w_init_.getOrigin().getZ(), tf::getYaw(T_w_init_.getRotation()));
  
  tf::Vector3 odom_p(o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z);
  tf::Vector3 p = T_w_init_ * odom_p;
  //ROS_INFO("odom_p: (%f,%f,%f)", odom_p.getX(), odom_p.getY(), odom_p.getZ());
  //ROS_INFO("p: (%f,%f,%f)", p.getX(), p.getY(), p.getZ());
  
  msg_.ob_ms.positions.clear();
  msg_.ob_ms.positions.push_back(p.getX());
  msg_.ob_ms.positions.push_back(p.getY());
  msg_.ob_ms.positions.push_back( utility_.displaceAngle(tf::getYaw(o.pose.pose.orientation), tf::getYaw(T_w_init_.getRotation())) );
  
  // Velocity should only be rotated - don't consider origin of T_w_init_
  tf::Vector3 odom_v(o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z);
  tf::Transform rot;
  rot.setOrigin( tf::Vector3(0,0,0) );
  rot.setRotation( T_w_init_.getRotation() );
  tf::Vector3 v = rot * odom_v;

  //ROS_INFO("odom_v: (%f,%f,%f) v: (%f,%f,%f)", odom_v.getX(), odom_v.getY(), odom_v.getZ(), v.getX(), v.getY(), v.getZ());

  msg_.ob_ms.velocities.clear();
  msg_.ob_ms.velocities.push_back(v.getX());
  msg_.ob_ms.velocities.push_back(v.getY());
  msg_.ob_ms.velocities.push_back(o.twist.twist.angular.z);


  // Set CircleGroup
  // Make a fitCir with fixed radius and 1 packedCir equal to fitCir
  msg_.cirGroup.fitCir.center.x = msg_.ob_ms.positions[0];
  msg_.cirGroup.fitCir.center.y = msg_.ob_ms.positions[1];
  msg_.cirGroup.fitCir.radius = 0.2;
  msg_.cirGroup.packedCirs.clear();
  msg_.cirGroup.packedCirs.push_back(msg_.cirGroup.fitCir);

  last_updated_ = ros::Time::now();
}

// No tf needed here, should already be in global coords
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

