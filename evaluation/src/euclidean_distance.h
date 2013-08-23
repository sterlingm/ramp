#ifndef EUCLIDEAN_DISTANCE_H
#define EUCLIDEAN_DISTANCE_H
#include "utility.h"
#include "ramp_msgs/Path.h"

class EuclideanDistance {
  public:
    EuclideanDistance() {}


    const double perform();

    ramp_msgs::Path path_;

};

#endif
