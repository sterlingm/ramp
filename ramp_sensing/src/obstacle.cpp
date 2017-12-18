#include "obstacle.h"

Obstacle::Obstacle() {}

Obstacle::~Obstacle() {}


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

