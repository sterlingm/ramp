#include "collision_detection.h"


CollisionDetection::CollisionDetection() {}

CollisionDetection::~CollisionDetection() 
{}

void CollisionDetection::init(ros::NodeHandle& h) {}




/** Returns true if trajectory_ is in collision with any of the objects */
const CollisionDetection::QueryResult CollisionDetection::perform() const 
{
  //ROS_INFO("In CollisionDetection::perform()");

  CollisionDetection::QueryResult result;
  
  
  //ROS_INFO("obstacle_trjs_.size(): %i", (int)obstacle_trjs_.size());
  // Predict the obstacle's trajectory
  for(uint8_t i=0;i<obstacle_trjs_.size();i++)
  {
    ramp_msgs::RampTrajectory ob_trajectory = obstacle_trjs_.at(i);

    // Query for collision
    CollisionDetection::QueryResult q = query(ob_trajectory);

    if(q.collision_)
    {
      result = q;
      break;
    }
  }


  //ROS_INFO("Exiting CollisionDetection::perform()");
  return result;  
} //End perform



/** 
 * This method returns true if there is collision between trajectory_ and the obstacle's trajectory, false otherwise 
 * The robots are treated as circles for simple collision detection
 */
const CollisionDetection::QueryResult CollisionDetection::query(const ramp_msgs::RampTrajectory ob_trajectory) const 
{
  //ROS_INFO("In CollisionDetection::query"); 
  //ROS_INFO("trajectory.points.size(): %i", (int)trajectory_.trajectory.points.size());
  //ROS_INFO("ob_trajectory.points.size(): %i", (int)ob_trajectory.trajectory.points.size());
  //ROS_INFO("ob_trajectory: %s", utility_.toString(ob_trajectory).c_str());

  double t_start = trajectory_.t_start.toSec();
  int j_offset = t_start * 10.f;
  //ROS_INFO("t_start: %f j_offset: %i", t_start, j_offset);

  CollisionDetection::QueryResult result;
  uint8_t t_checkColl = 1;

  /*if(ob_trajectory.trajectory.points.size() <= 2) {
    if(id == 0)
      std::cout<<"\nRobot 1 has no trajectory!\n";
    else  
      std::cout<<"\nRobot 0 has no trajectory!\n";
  }*/


  if(trajectory_.trajectory.points.size() == 1 && ob_trajectory.trajectory.points.size() == 1)
  {

  }
  
  uint16_t i_stop;
  if(trajectory_.i_knotPoints.size() == 0) 
  {
    //ROS_INFO("i_stop=0");
    i_stop = 0;
  }
  else if(trajectory_.i_knotPoints.size() == 1) 
  {
    //ROS_INFO("i_stop=1");
    i_stop = 1;
  }
  else if(trajectory_.i_knotPoints.size() <= 2) 
  {
    //ROS_INFO("i_stop=kp 1");
    i_stop = trajectory_.i_knotPoints.at(1);
  }
  else 
  {
    //ROS_INFO("i_stop=kp 2");
    i_stop = trajectory_.i_knotPoints.at(2);
  }

  int j_start;
  
  //ROS_INFO("i_stop: %i", i_stop);
  
  // For every point, check circle detection on a subset of the obstacle's trajectory
  float radius = 0.18f;
  for(uint16_t i=0;i<i_stop;i++) 
  {
    
    // Get the ith point on the trajectory
    trajectory_msgs::JointTrajectoryPoint p_i = trajectory_.trajectory.points.at(i);

    
    // Compute which point on the obstacle trajectory to start doing collision checking
    if(ob_trajectory.trajectory.points.size() == 1)
    {
      j_start = 0;
    }
    else if(i <= t_checkColl)
    {
      j_start = 0+j_offset;
    }
    else
    {
      j_start = (i-t_checkColl)+j_offset;
    }

    // *** Test position i for collision against some points on obstacle's trajectory ***
    // Obstacle trajectory should already be in world coordinates!
    for(int j = j_start; 
        j<=(i+t_checkColl+j_offset) && j<ob_trajectory.trajectory.points.size(); 
        j++) 
    {
      //ROS_INFO("i: %i j: %i", i, j);

      // Get the jth point of the obstacle's trajectory
      trajectory_msgs::JointTrajectoryPoint p_ob  = ob_trajectory.trajectory.points.at(j);

      // Get the distance between the centers
      float dist = sqrt( pow(p_i.positions.at(0) - p_ob.positions.at(0),2) + pow(p_i.positions.at(1) - p_ob.positions.at(1),2) );

      /*ROS_INFO("Comparing trajectory point (%f,%f) and obstacle point (%f,%f): dist = %f", 
          p_i.positions.at(0), p_i.positions.at(1), 
          p_ob.positions.at(0), p_ob.positions.at(1), 
          dist);*/
      
        

      // If the distance between the two centers is less than the sum of the two radii, 
      // there is collision
      if( dist <= radius*2 ) 
      {
        /*ROS_INFO("Points in collision: (%f,%f), and (%f,%f), dist: %f i: %i j: %i",
            p_i.positions.at(0),
            p_i.positions.at(1),
            p_ob.positions.at(0),
            p_ob.positions.at(1),
            dist,
            (int)i,
            (int)j);*/
        result.collision_ = true;
        result.t_firstCollision_ = p_i.time_from_start.toSec();
        i = i_stop;
        break;
      } // end if
    } // end for
  } // end for


  //ROS_INFO("Exiting CollisionDetection::query");
  return result;
} //End query








