#include "ros/ros.h"
#include "utility.h"
#include "knot_point.h"
#include "ramp_msgs/ObstacleList.h"
#include "nav_msgs/Odometry.h"
#include "range.h"
#include "ramp_trajectory.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "tf/transform_datatypes.h"


/*
 * Indices of "ranges" argument should be:
 * 0 - x position range
 * 1 - y position range
 * 2 - orientation position range
 * 3 - linear velocity range
 * 4 - angular velocity range
 */
const ramp_msgs::Obstacle buildObstacleMsg(const std::vector<Range> ranges)
{
  ramp_msgs::Obstacle result;

  nav_msgs::Odometry odom_msg;

  // Set the x,y position
  odom_msg.pose.pose.position.x = ranges[0].random(); 
  odom_msg.pose.pose.position.y = ranges[1].random(); 
  odom_msg.pose.pose.position.z = 0;

  // Set orientation
  double theta                    = ranges[2].random();
  odom_msg.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(theta);
  
  // Get random values for linear and angular velocities
  double v = ranges[3].random();
  double w = ranges[4].random();

  // For linear velocity, calculate x and y components
  double v_x = v*cos(tf::getYaw(odom_msg.pose.pose.orientation));
  double v_y = v*sin(tf::getYaw(odom_msg.pose.pose.orientation));;

  // Set velocities
  odom_msg.twist.twist.linear.x   = v_x;
  odom_msg.twist.twist.linear.y   = v_y;
  odom_msg.twist.twist.angular.z  = ranges[5].random();

  result.odom_t = odom_msg;
  return result;
}




/*
 * Indices of "ranges" argument should be:
 * 0 - x position range
 * 1 - y position range
 * 2 - orientation position range
 * 3 - linear velocity range
 * 4 - angular velocity range
 */
const ramp_msgs::TrajectoryRequest buildObstacleTrajReq(const std::vector<Range> ranges)
{
  ramp_msgs::TrajectoryRequest result;

  // Get random position
  KnotPoint kp;
  kp.motionState_.msg_.positions.push_back(ranges[0].random());
  kp.motionState_.msg_.positions.push_back(ranges[1].random());
  kp.motionState_.msg_.positions.push_back(ranges[2].random());

  // Generate random velocities
  double v = ranges[3].random();
  double w = ranges[4].random();

  double v_x = v*cos(kp.motionState_.msg_.positions[2]);
  double v_y = v*sin(kp.motionState_.msg_.positions[2]);

  kp.motionState_.msg_.velocities.push_back(v_x);
  kp.motionState_.msg_.velocities.push_back(v_y);
  kp.motionState_.msg_.velocities.push_back(w);

  result.path.points.push_back(kp.buildKnotPointMsg());
  result.type = PREDICTION;

  return result;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_world_state");
  ros::NodeHandle handle;
  Utility utility;

  srand(time(NULL));
    
  // Create Ranges for each dimension
  Range x_pos(0, 3.5);
  Range y_pos(0, 3.5);
  Range theta_pos(-PI, PI);
  Range linear_vels(-0.33, 0.33);
  Range angular_vels(-PI/4.f, PI/4.f);

  // Push those Ranges onto a vector
  std::vector<Range> ranges;
  ranges.push_back(x_pos);
  ranges.push_back(y_pos);
  ranges.push_back(theta_pos);
  ranges.push_back(linear_vels);
  ranges.push_back(angular_vels);

  // Set the number of obstacles (given by command line argument)
  const int NUM_OBSTACLES = argc > 1 ? atoi(argv[1]) : 1;

  ROS_INFO("NUM_OBSTACLES: %i", NUM_OBSTACLES);

  /*
   * Population an ObstacleList object
   * Create x number of random obstacles, x=NUM_OBSTACLES
   */
  ramp_msgs::ObstacleList obs;
  for(uint8_t i=0;i<NUM_OBSTACLES;i++)
  {
    obs.obstacles.push_back( buildObstacleMsg(ranges) ); 
  }
  ROS_INFO("obs.obstacles.size(): %i", (int)obs.obstacles.size());

  // Create Publisher for the ObstacleList
  ros::Publisher pub_obs = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 100);

  // Sleep briefly
  ros::Duration d(0.5f);
  d.sleep();

  // Publish the list of obstacles
  pub_obs.publish(obs);
  //pub_obs.publish(obs);







  /*
   * Below is code to generate the trajectories and work with the trajectories directly
   * The trajectory_generator node MUST BE RUNNING OR THIS WILL FAIL
   * In a separate terminal, run "rosrun trajectory_generator trajectory_generator" to start that node
   */

  /*

  // Populate the srv message with TrajectoryRequests
  ramp_msgs::TrajectorySrv tr_srv;
  for(uint i=0;i<NUM_OBSTACLES;i++)
  {
    tr_srv.request.reqs.push_back( buildObstacleTrajReq(ranges) );
  }

  // Create a service client to call the trajectory generator
  ros::ServiceClient traj_client = handle.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");

  // Create vector to hold obstacle trajectories
  std::vector<RampTrajectory> ob_trajecs;

  // Call the trajectory_generator package
  if(traj_client.call(tr_srv))
  {
    for(uint8_t i=0;i<tr_srv.response.resps.size();i++)
    {
      ob_trajecs.push_back(tr_srv.response.resps[i].trajectory);
    }
  }
  else
  {
    ROS_ERROR("Some error(s) getting trajectories");
  }


  // If no errors, then the trajectories are in ob_trajecs

  */
   

  return 0;
}
