#include "trajectory_request_handler.h"


TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : handle_(h) 
{
  client_ = handle_.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");
  ROS_INFO("In constructor, client.isValid: %s", client_.isValid() ? "True" : "False");
}


const bool TrajectoryRequestHandler::request(ramp_msgs::TrajectorySrv& tr) 
{
  ROS_INFO("In request, client.isValid: %s", client_.isValid() ? "True" : "False");
  return client_.call(tr);
}
