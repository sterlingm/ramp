#include "insert.h"


Insert::Insert(const ramp_msgs::Path p) : path_(p) {}
Insert::Insert(const nav_msgs::Path p) : navPath_(p) {}



const ramp_msgs::Path Insert::perform() 
{
  if(path_.points.size() < 7)
  {
    // Randomly choose two adjacent knot points
    // One index is generated randomly and the second will be the next knot point,
    // unless the generated index is the last knot point index
    // The new knot point will be inserted between those two knot points
    unsigned int i_knotPoint1 = rand() % path_.points.size(); 
    unsigned int i_knotPoint2 = (i_knotPoint1 == path_.points.size()-1) ? i_knotPoint1-1 : i_knotPoint1+1;

    // If the last index was chosen, swap the values so i_knotPoint1 < i_knotPoint2
    // this just makes it simpler 
    if(i_knotPoint2 < i_knotPoint1) 
    {
      unsigned int swap = i_knotPoint1;
      i_knotPoint1 = i_knotPoint2;
      i_knotPoint2 = swap;
    }


    // Generate a new, random motion state
    ramp_msgs::KnotPoint kp;
    while(!checkConstraints_.validKPForPath(kp, path_))
    {
      kp.motionState.positions.clear();
      kp.motionState.velocities.clear();
      
      for(unsigned int i=0;i<path_.points.at(0).motionState.positions.size();i++) {
        
        // Generate a random value for each K in the specified range
        double  min = ranges_.at(i).min;
        double  max = ranges_.at(i).max;
        float r = (float)rand();
        //ROS_INFO("r: %f min: %f max: %f max-min: %f", r, min, max, max-min);
        float   temp = (min == 0 && max == 0) ? 0 :      
               ( min + r / ((float)RAND_MAX / (max - min)) );

        // Set the position and velocity
        kp.motionState.positions.push_back(temp);
        kp.motionState.velocities.push_back(0);
      }
    }

    // Insert the configuration 
    path_.points.insert(path_.points.begin()+i_knotPoint2, kp);
  }

  return path_; 
}

const nav_msgs::Path Insert::navPerform() 
{
	if(navPath_.poses.size() < 7)
  {
    // Randomly choose two adjacent points
    // One index is generated randomly and the second will be the next point,
    // unless the generated index is the last point index
    // The new point will be inserted between those two points
    unsigned int i_point1 = rand() % navPath_.poses.size(); 
    unsigned int i_point2 = (i_point1 == navPath_.poses.size()-1) ? i_point1-1 : i_point1+1;

    // If the last index was chosen, swap the values so i_knotPoint1 < i_knotPoint2
    // this just makes it simpler 
    if(i_point2 < i_point1) 
    {
      unsigned int swap = i_point1;
      i_point1 = i_point2;
      i_point2 = swap;
    }


    // Generate a new PoseStamped
    geometry_msgs::PoseStamped p;

    while(!checkConstraints_.validPoseForPath(p, navPath_))
    {
			p.pose.position.x = 0.0;
			p.pose.position.y = 0.0;
			p.pose.position.z = 0.0;
      
      for(unsigned int i=0;i<3;i++) {
        
        // Generate a random value for each K in the specified range
        double  min = ranges_.at(i).min;
        double  max = ranges_.at(i).max;
        float r = (float)rand();
        //ROS_INFO("r: %f min: %f max: %f max-min: %f", r, min, max, max-min);
        float   temp = (min == 0 && max == 0) ? 0 :      
               ( min + r / ((float)RAND_MAX / (max - min)) );

        // Set the position and velocity
        if (i == 0)
				{
					p.pose.position.x = temp;
				}
				else if (i == 1)
				{
					p.pose.position.y = temp;
				}
				else 
				{
					p.pose.position.z = temp;
				}
      }
    }

    // Insert the configuration 
    navPath_.poses.insert(navPath_.poses.begin()+i_point2, p);
  }
	
  return navPath_;
}
