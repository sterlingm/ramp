#ifndef RVIZ_HANDLER_H
#define RVIZ_HANDLER_H
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>

class RvizHandler {
  public:
    RvizHandler(const ros::NodeHandle& h);

    void sendMarkerArray(const visualization_msgs::MarkerArray& ma);
    void sendRobotPose(const visualization_msgs::MarkerArray& ma);

  private:
    ros::NodeHandle handle_;
    ros::Publisher pub_markerArray_;
    ros::Publisher pub_robot_pose_;
    ros::Publisher pub_obs_once_;
};

#endif
