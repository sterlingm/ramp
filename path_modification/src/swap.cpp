#include "swap.h"


Swap::Swap(const ramp_msgs::Path p) : path_(p) {}
Swap::Swap(const nav_msgs::Path p) : navPath_(p) {}


/** Swap does NOT swap the velocity values! Only the positions! */
const ramp_msgs::Path Swap::perform() 
{

  if(path_.points.size() > 3) 
  {
    unsigned int i_knotPoint1 = rand() % (path_.points.size()-2) + 1;
    unsigned int i_knotPoint2; 
    
    // Make sure the two configurations are different
    do 
    {
      i_knotPoint2 = rand() % (path_.points.size()-2) + 1;
    } while(i_knotPoint1 == i_knotPoint2);

    //std::cout<<"\ni_knotPoint1: "<<i_knotPoint1<<" i_knotPoint2: "<<i_knotPoint2;
    
    // Swap the knot points' positions
    std::vector<double> temp = path_.points.at(i_knotPoint1).motionState.positions;
    path_.points.at(i_knotPoint1).motionState.positions = path_.points.at(i_knotPoint2).motionState.positions;
    path_.points.at(i_knotPoint2).motionState.positions = temp;

  }

  return path_;
}


/** Swap does NOT swap the velocity values! Only the positions! */
const nav_msgs::Path Swap::navPerform() 
{

  if(navPath_.poses.size() > 3) 
  {
    unsigned int i_point1 = rand() % (navPath_.poses.size()-2) + 1;
    unsigned int i_point2; 
    
    // Make sure the two configurations are different
    do 
    {
      i_point2 = rand() % (navPath_.poses.size()-2) + 1;
    } while(i_point1 == i_point2);
    
    // Swap the points' positions
    geometry_msgs::PoseStamped temp = navPath_.poses[i_point1];
    //std::vector<double> temp = navPath_.poses.at(i_Point1);
    navPath_.poses[i_point1] = navPath_.poses[i_point2];
    navPath_.poses[i_point2] = temp;

  }

  return navPath_;
}
