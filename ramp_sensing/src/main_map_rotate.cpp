#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;



void sendTF(const ros::TimerEvent& e)
{
  ROS_INFO("In sendTF");

  // Create broadcaster
  static tf::TransformBroadcaster br;

  // Create transform
  tf::Transform transform;

  // Set position vector
  transform.setOrigin( tf::Vector3(0, 0, 0) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(0, 0, 0.9);
  transform.setRotation(q);

  ROS_INFO("Time::now(): %f", ros::Time::now().toSec());

  // Broadcast to tf
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now()+ros::Duration(0.05), "map", "costmap"));
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle handle;

  ros::Timer tf_timer = handle.createTimer(ros::Duration(0.1), &sendTF);

  ROS_INFO("Timer created");

  ros::spin();
  return 0;
}
