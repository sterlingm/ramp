#include "ros/ros.h"
#include <signal.h>
#include "mobile_robot.h"
#include "ramp_msgs/MotionState.h"

MobileRobot robot;

void trajCallback(const ramp_msgs::RampTrajectory::ConstPtr& msg) {
  //std::cout<<"\nGot the trajectory message!\n";
  //std::cout<<"\nPress enter to call updateTrajectory\n";
  //std::cin.get();
  robot.updateTrajectory(*msg);
}






/** Initialize the MobileRobot's publishers and subscibers*/
void init_advertisers_subscribers(MobileRobot& robot, ros::NodeHandle& handle, bool simulation) 
{

  
  // Publishers
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(MobileRobot::TOPIC_STR_TWIST, 1000);
  robot.pub_update_ = handle.advertise<ramp_msgs::MotionState>(MobileRobot::TOPIC_STR_UPDATE, 1000);

  if(simulation) 
  {
    robot.pub_cmd_vel_ = handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }
 
  // Subscribers
  robot.sub_odometry_ = handle.subscribe(MobileRobot::TOPIC_STR_ODOMETRY, 1, &MobileRobot::odomCb, &robot);
  robot.sub_imminent_collision_ = handle.subscribe(MobileRobot::TOPIC_STR_IC, 1, &MobileRobot::imminentCollisionCb, &robot);

  // Timers
  // 15 Hz seems to be the fastest possible while avoiding nan errors
  robot.timer_ = handle.createTimer(ros::Duration(1.f / 30.f), &MobileRobot::updateCallback, &robot);
} // End init_advertisers_subscribers



void reportData(int sig)
{
  double sum = 0;
  for(int i=0;i<robot.t_points_.size();i++)
  {
    sum += robot.t_points_[i].toSec();
  }
  ROS_INFO("Average point time: %f", (sum / robot.t_points_.size()));

  ros::shutdown();
}



/*void readParam(ros::NodeHandle& handle)
{
  handle_local.param("orientation", robot.initial_theta_, 0.);
  std::cout<<"\n*********robot.orientation: "<<robot.initial_theta_;
  
  bool sim=false;
  handle_local.param("simulation", sim, true);
  std::cout<<"\nsim: "<<sim<<"\n";
  robot.sim_ = sim;
 
 
  bool check_imminent_coll=true;
  handle_local.param("check_imminent_coll", check_imminent_coll, true);
  ROS_INFO("check_imminent_coll: %s", check_imminent_coll ? "True" : "False");
  robot.check_imminent_coll_ = check_imminent_coll;
}*/



int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ramp_control");


  ros::NodeHandle handle;  
  ros::NodeHandle handle_local("~");

  ros::Subscriber sub_traj = handle.subscribe("bestTrajec", 1, trajCallback);

  setvbuf(stdout, NULL, _IOLBF, 4096);
 
  //handle.param("ramp_control/orientation", robot.initial_theta_, 0.785);
  handle_local.param("orientation", robot.initial_theta_, -0.785);
  std::cout<<"\n*********robot.orientation: "<<robot.initial_theta_;

  bool sim=false;
  handle_local.param("simulation", sim, true);
  std::cout<<"\nsim: "<<sim<<"\n";
  robot.sim_ = sim;
 
 
  bool check_imminent_coll=true;
  handle_local.param("check_imminent_coll", check_imminent_coll, true);
  ROS_INFO("check_imminent_coll: %s", check_imminent_coll ? "True" : "False");
  robot.check_imminent_coll_ = check_imminent_coll;


  std::string global_frame;
  if(handle.hasParam("/ramp/global_frame"))
  {
    handle.getParam("/ramp/global_frame", global_frame);
    ROS_INFO("global_frame: %s", global_frame.c_str());
  }
  else
  {
    ROS_ERROR("Could not find rosparam ramp/global_frame");
  }


  /*
   * Get transform information from odom to the frame the planner uses
   */
  ros::Duration d(0.5);
  d.sleep();

  tf::TransformListener listen;
  listen.waitForTransform(global_frame, "odom", ros::Time(0), ros::Duration(2.0));
  listen.lookupTransform(global_frame, "odom", ros::Time(0), robot.tf_global_odom_);
  ROS_INFO("(ramp_control) Odom tf: translate: (%f, %f) rotation: %f", robot.tf_global_odom_.getOrigin().getX(), robot.tf_global_odom_.getOrigin().getX(), robot.tf_global_odom_.getRotation().getAngle());
  robot.tf_rot_ = robot.tf_global_odom_.getRotation().getAngle();
  



  // Initialize publishers and subscribers
  init_advertisers_subscribers(robot, handle, sim);



  // Make a blank ramp_msgs::RampTrajectory
  ramp_msgs::RampTrajectory init;
  robot.trajectory_ = init;
  
  signal(SIGINT, reportData);

  // Put a rate on the while loop to prevent high CPU usage
  // With no rate, CPU usage is ~110%
  // With rate even as high as 1000, goes down to ~10-15%
  // Shouldn't have problem moving right away on a trajectory with 1000Hz as the rate
  ros::Rate r(1000);
  while(ros::ok()) 
  {
    robot.moveOnTrajectory();
    r.sleep();
    ros::spinOnce();
  }

  fflush(stdout);

  std::cout<<"\nExiting Normally\n";
  return 0;
}
