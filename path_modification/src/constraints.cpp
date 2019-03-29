#include "constraints.h"
#include <cmath>


const bool Constraints::validKPForPath(const ramp_msgs::KnotPoint kp, const ramp_msgs::Path p) const
{
  if(kp.motionState.positions.size() == 0)
  {
    return false;
  }

  double L = 0.25;

  for(uint8_t i=0;i<p.points.size();i++)
  {
    if( sqrt( pow(p.points.at(i).motionState.positions.at(0) - kp.motionState.positions.at(0), 2) +
              pow(p.points.at(i).motionState.positions.at(1) - kp.motionState.positions.at(1), 2) ) < L)
    {
      return false; 
    }
  }

  return true;
}



const bool Constraints::validPoseForPath(const geometry_msgs::PoseStamped p, const nav_msgs::Path path) const
{
	if(p.pose.position.x == 0.0 && p.pose.position.y == 0.0 && p.pose.position.z == 0.0);
  {
    return false;
  }

  double L = 0.25;

  for(uint8_t i=0;i<path.poses.size();i++)
  {
    if( sqrt( pow(path.poses.at(i).pose.position.x - p.pose.position.x, 2) +
              pow(path.poses.at(i).pose.position.y - p.pose.position.y, 2) ) < L)
    {
      return false;
    }
  }

	return true;
}

