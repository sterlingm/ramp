#include <iostream>
#include <ros/ros.h>
#include "ramp_msgs/TrajectorySrv.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "utility.h"

Utility u;
ramp_msgs::RampTrajectory ob_trj;
ros::Time t_start;
ros::ServiceClient gz_set_model;
bool first=true;

/*
 * Publish obstacle information at 20Hz to simulate sensing cycles
 */
void pubObTrj(const ros::TimerEvent e)
{
  //ROS_INFO("In pubObTrj");

  ros::Duration d_elapsed = ros::Time::now() - t_start;
  
  int index = d_elapsed.toSec()*10;
  ROS_INFO("t_start: %f ros::Time::now(): %f d_elapsed: %f index: %i", t_start.toSec(), ros::Time::now().toSec(), d_elapsed.toSec(), index);
  
  // Get the point
  trajectory_msgs::JointTrajectoryPoint p = index >= ob_trj.trajectory.points.size() ? ob_trj.trajectory.points[ob_trj.trajectory.points.size()-1] : ob_trj.trajectory.points[index]; 

  // Build srv to send
  gazebo_msgs::SetModelState ms; 
  ms.request.model_state.model_name = "cardboard_box";
  ms.request.model_state.pose.position.x = p.positions[0];
  ms.request.model_state.pose.position.y = p.positions[1];

  // Call srv
  if(!gz_set_model.call(ms))
  {
    ROS_ERROR("Error calling set_model_state");
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_gazebo_obs");
  ros::NodeHandle handle;

  // Connect to trajectory generator
  ros::ServiceClient client       = handle.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");
  ros::ServiceClient gz_get_model = handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gz_set_model = handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // Build a trajectory request
  ramp_msgs::TrajectoryRequest tr;

  // Build a path
  ramp_msgs::Path p;

  // Get initial state of the object
  std::string model_name = "cardboard_box";
  gazebo_msgs::GetModelState cb;
  cb.request.model_name = model_name;

  if(gz_get_model.call(cb))
  {
    ROS_INFO("%s pose: position (%f,%f,%f) orientation [%f,%f,%f,%f]", model_name.c_str(), cb.response.pose.position.x, cb.response.pose.position.y, cb.response.pose.position.z, cb.response.pose.orientation.x, cb.response.pose.orientation.y, cb.response.pose.orientation.z, cb.response.pose.orientation.w);
  }
  else
  {
    ROS_ERROR("Error calling get_model_state");
  }



  /*
   * 
   */

  // Build the knotpoints of the path
  ramp_msgs::KnotPoint kp1;
  kp1.motionState.positions.push_back(cb.response.pose.position.x);
  kp1.motionState.positions.push_back(cb.response.pose.position.y);

  // Make sure deltaX > deltaY
  ramp_msgs::KnotPoint kp2;
  kp2.motionState.positions.push_back(cb.response.pose.position.x-8);
  kp2.motionState.positions.push_back(cb.response.pose.position.y);
  kp2.motionState.positions.push_back(PI);

  double theta1 = u.findAngleFromAToB(kp1.motionState.positions, kp2.motionState.positions);
  kp1.motionState.positions.push_back(PI);


  ramp_msgs::KnotPoint kp3;
  kp3.motionState.positions.push_back(cb.response.pose.position.x-2.0);
  kp3.motionState.positions.push_back(cb.response.pose.position.y);
  kp3.motionState.positions.push_back(PI);

  // Push knotpoints onto the path
  p.points.push_back(kp1);
  p.points.push_back(kp2);
  //p.points.push_back(kp3);
  

  // Build the request
  tr.path = p;
  tr.type = HOLONOMIC;

  // Build the srv
  ramp_msgs::TrajectorySrv ts;
  ts.request.reqs.push_back(tr);

  // Call trajectory_generator
  if(client.call(ts))
  {
    ROS_INFO("Trajectory: %s", u.toString(ts.response.resps[0].trajectory).c_str());
    ob_trj = ts.response.resps[0].trajectory;
  }
  else
  {
    ROS_ERROR("Error calling trajectory_generator");
  }
  
  // Make Timer to start moving
  //ros::Duration d(2.0);
  //d.sleep();

  // Wait for ramp to start moving the robot
  ros::Rate r(25);
  bool cc_started = false;
  while(!cc_started)
  {
    handle.getParam("/ramp/cc_started", cc_started);
    //ROS_INFO("/ramp/cc_started: %s", cc_started ? "True" : "False");
    r.sleep();
    ros::spinOnce();
  }

  t_start = ros::Time::now();
  ROS_INFO("Initial t_start: %f", t_start.toSec());
  ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());
  ros::Timer pub_ob_trj = handle.createTimer(ros::Duration(1.0f/20.0f), pubObTrj);
  ros::spin();

  printf("\nExiting normally\n");
  return 0;
}
