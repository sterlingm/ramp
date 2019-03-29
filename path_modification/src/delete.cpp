#include "delete.h"

Delete::Delete(const ramp_msgs::Path p) : path_(p) {}
Delete::Delete(const nav_msgs::Path p) : navPath_(p) {}


const ramp_msgs::Path Delete::perform() {

  if(path_.points.size() > 2) 
  {
 
    // Randomly choose a knot point to delete 
    // Cannot delete the start or goal, so adjust the range a bit, range= [1,(size-2)]
    unsigned int i_knotPoint = rand() % (path_.points.size()-2) + 1; 

    // Delete the knot point
    path_.points.erase(path_.points.begin()+i_knotPoint);
  }

  return path_;
}



const nav_msgs::Path Delete::navPerform() {

  if(navPath_.poses.size() > 2) 
  {
 
    // Randomly choose a knot point to delete 
    // Cannot delete the start or goal, so adjust the range a bit, range= [1,(size-2)]
    unsigned int i_point = rand() % (navPath_.poses.size()-2) + 1; 

    // Delete the knot point
    navPath_.poses.erase(navPath_.poses.begin()+i_point);
  }

  return navPath_;
}
