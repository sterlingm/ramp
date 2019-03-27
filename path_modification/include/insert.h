#ifndef INSERT_H
#define INSERT_H
#include "utility.h"
#include "constraints.h"

class Insert {
  public:
    Insert() {}
    Insert(const ramp_msgs::Path p); 
    Insert(const nav_msgs::Path p);

    const ramp_msgs::Path perform();
    const nav_msgs::Path navPerform();

    ramp_msgs::Path path_;
    nav_msgs::Path navPath_;
    std::vector<ramp_msgs::Range> ranges_;
    Utility utility_;
  private:
    Constraints checkConstraints_;
};

#endif
