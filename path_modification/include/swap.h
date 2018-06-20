#ifndef SWAP_H
#define SWAP_H
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Range.h"

class Swap {
  public:
    Swap() {} 
    Swap(const ramp_msgs::Path p);
   
    const ramp_msgs::Path perform();
   
    std::vector<ramp_msgs::Range> ranges_;
    ramp_msgs::Path path_; 
};

#endif
