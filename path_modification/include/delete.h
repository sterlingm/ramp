#ifndef DELETE_H
#define DELETE_H
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Range.h"

class Delete {
  public:
    Delete() {}
    Delete(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();
    
    std::vector<ramp_msgs::Range> ranges_;
    ramp_msgs::Path path_;
  
  private:

};

#endif 
