#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist;

// Turn the robot on Twist t for Duration d
void turn(geometry_msgs::Twist t, ros::Duration d)
{
  ros::Rate r(10);
  ros::Time t_start = ros::Time::now();
  while( (ros::Time::now() - t_start) < d )
  {
    pub_twist.publish(t); 
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clear_obs");
  ros::NodeHandle handle;

  pub_twist = handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

  // Create twist messages to rotate
  geometry_msgs::Twist t_left, t_right;
  t_left.angular.z = 0.707;
  t_right.angular.z = -0.707;

  // Initialize duration to turn in each direction
  ros::Duration d(2.0);

  // Run until process is killed
  while(ros::ok())
  {
    // Turn left
    turn(t_left, d);

    // Turn right
    turn(t_right, d);
  }

  printf("\nExiting normally\n");
  return 0;
}
