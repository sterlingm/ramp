#include "ros/ros.h"
#include "trajectory.h"
#include "subscribe_and_publish.h"
#include "ramp_msgs/TrajectoryRequest.h"

ros::Publisher  pub_trajs;
ros::Subscriber sub_paths;


int main(int argc, char** argv) {
  

  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle handle;

  SubscribeAndPublish sap(handle);

  /** The following is just for testing purposes. */

  //Create some knot points
  /*geometry_msgs::Pose2D kp_start;
  kp_start.x = 0;
  kp_start.y = 0;
  kp_start.theta = 50;

  geometry_msgs::Pose2D kp_mid;
  kp_mid.x = 1;
  kp_mid.y = 0;
  kp_mid.theta = 50;

  geometry_msgs::Pose2D kp_end;
  kp_end.x = 0.5;
  kp_end.y = 0;
  kp_end.theta = 50;

  //Create a trajectory
  Trajectory traj;
  
  //set the resolution rate
  traj.resolutionRate_ = 5;
  
  //Set knot points
  traj.knot_points_.push_back(kp_start);;
  traj.knot_points_.push_back(kp_mid);
  traj.knot_points_.push_back(kp_end);

  //Set times
  traj.t_.push_back(2);
  traj.t_.push_back(1);
   
  

  //Generate the trajectory
  std::vector<MotionState> Ms = traj.generate();
  
  //Build a Trajectory msg
  ramp_msgs::Trajectory msg = traj.buildTrajectoryMsg();
  

  std::cout<<"\nTrajectory:"<<traj.toString();

  //Publish the message and quit
  std::cout<<"\nPress Enter to publish!\n";
  std::cin.get();

  pub_trajs.publish(msg);
  std::cout<<"\nMessage published!";*/


  std::cout<<"\nSpinning...\n";  
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
