#ifndef DELETE_H
#define DELETE_H
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Range.h"
#include "nav_msgs/Path.h"

class Delete {
  public:
    Delete() {}
    Delete(const ramp_msgs::Path p);
		Delete(const nav_msgs::Path p);

    const ramp_msgs::Path perform();
		const nav_msgs::Path navPerform();
    
    std::vector<ramp_msgs::Range> ranges_;
    ramp_msgs::Path path_;
		nav_msgs::Path navPath_;
  
  private:

};

#endif 
