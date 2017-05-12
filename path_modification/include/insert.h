#ifndef INSERT_H
#define INSERT_H
#include "utility.h"
#include "constraints.h"

class Insert {
  public:
    Insert() {}
    Insert(const ramp_msgs::Path p); 

    const ramp_msgs::Path perform();

    ramp_msgs::Path path_;
    Utility utility_;
  private:
    Constraints checkConstraints_;
};

#endif
