#include "rviz_handler.h"

RvizHandler::RvizHandler(const ros::NodeHandle& h) : handle_(h) 
{
  pub_markerArray_ = handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  pub_robot_pose_ = handle_.advertise<visualization_msgs::MarkerArray>("visual_robot_pose", 1);
}

void RvizHandler::sendMarkerArray(const visualization_msgs::MarkerArray& ma)
{
  pub_markerArray_.publish(ma);
}

void RvizHandler::sendRobotPose(const visualization_msgs::MarkerArray& ma)
{
  pub_robot_pose_.publish(ma);
}