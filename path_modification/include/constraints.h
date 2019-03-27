#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H
#include "utility.h"


class Constraints
{
  public:
    Constraints() {}

    const bool validKPForPath(const ramp_msgs::KnotPoint kp, const ramp_msgs::Path p) const;
    const bool validPoseForPath(const geometry_msgs::PoseStamped p, const nav_msgs::Path path) const;
};

#endif
