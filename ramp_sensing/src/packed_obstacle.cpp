#include "packed_obstacle.h"

PackedObstacle::PackedObstacle() {
}

PackedObstacle::PackedObstacle(const std::vector<Circle> cirs) 
{
  for(int i=0;i<cirs.size();i++)
  {
    ramp_msgs::Circle c;
    c.center.x  = cirs[i].center.x;
    c.center.y  = cirs[i].center.y;
    c.radius    = cirs[i].radius;

    msg_.circles.push_back(c);
  }
}

PackedObstacle::~PackedObstacle() {}
