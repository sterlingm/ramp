#include "ros/ros.h"
#include "planner.h"
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
 

Utility utility;


int                 id;
MotionState         start, goal;
std::vector<Range>  ranges;
double              radius;
double              max_speed_linear;
double              max_speed_angular;
int                 population_size;
int                 gensBeforeCC;
bool                sub_populations;
bool                modifications;
bool                evaluations;
bool                seedPopulation;
bool                errorReduction;
bool                only_sensing;
bool                moving_robot;
bool                shrink_ranges;
double              t_cc_rate;
double              t_sc_rate;
int                 pop_type;
TrajectoryType      pt;
std::vector<std::string> ob_topics;
std::string         global_frame;
std::string         update_topic;

float costmap_width, costmap_height, costmap_origin_x, costmap_origin_y;
ros::Publisher pub_rviz;

bool use_start_param;
bool start_planner;


// Initializes a vector of Ranges that the Planner is initialized with
// Must be called AFTER radius is set
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  for(unsigned int i=0;i<dof_min.size();i++) 
  {
    Range temp(dof_min.at(i), dof_max.at(i));
    //if(global_frame != "odom" && (i == 0 || i == 1))
    if(shrink_ranges)
    {
      temp.msg_.min += radius;
      temp.msg_.max -= radius;
    }
    ranges.push_back(temp); 
  }

} // End initDOF



// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) 
{
  for(unsigned int i=0;i<s.size();i++) {
    start.msg_.positions.push_back(s.at(i));
    goal.msg_.positions.push_back(g.at(i));

    start.msg_.velocities.push_back(0);
    goal.msg_.velocities.push_back(0);

    start.msg_.accelerations.push_back(0);
    goal.msg_.accelerations.push_back(0);

    start.msg_.jerks.push_back(0);
    goal.msg_.jerks.push_back(0);
  }
} // End initStartGoal




/** Loads all of the ros parameters from .yaml 
 *  Calls initDOF, initStartGoal */
void loadParameters(const ros::NodeHandle handle)
{
  ros::NodeHandle _h;

  std::cout<<"\nLoading parameters\n";
  std::cout<<"\nHandle NS: "<<handle.getNamespace();

  std::string key;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) 
  {
    handle.getParam("robot_info/id", id);
  }
  else 
  {
    //ROS_ERROR("Did not find parameter robot_info/id");
  }

  // Get the radius of the robot
  if(handle.hasParam("robot_info/radius")) 
  {
    handle.getParam("robot_info/radius", radius);
  }
  else 
  {
    //ROS_ERROR("Did not find parameter robot_info/radius");
  }

  if(handle.hasParam("ramp/global_frame"))
  {
    handle.getParam("ramp/global_frame", global_frame);
    //ROS_INFO("global_frame: %s", global_frame.c_str());
  }
  else
  {
    //ROS_ERROR("Could not find rosparam ramp/global_frame");
  }

  if(handle.hasParam("ramp/update_topic"))
  {
    handle.getParam("ramp/update_topic", update_topic);
    //ROS_INFO("update_topic: %s", update_topic.c_str());
  }
  else
  {
    //ROS_ERROR("Could not find rosparam ramp/update_topic");
  }
  
  if(handle.hasParam("ramp/shrink_ranges"))
  {
    handle.getParam("ramp/shrink_ranges", shrink_ranges);
    std::cout<<"\nshrink_ranges: "<<shrink_ranges;
  }

  if(handle.hasParam("ramp/use_start_param"))
  {
    handle.getParam("ramp/use_start_param", use_start_param);
    std::cout<<"\nuse_start_param: "<<use_start_param;
  }


  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    initDOF(dof_min, dof_max);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
    exit(1);
  }

  if(handle.hasParam("robot_info/max_speed_linear"))
  {
    handle.getParam("robot_info/max_speed_linear", max_speed_linear);
  }
  else
  {
    ROS_ERROR("Did not find robot_info/max_speed_linear rosparam");
    exit(1);
  }

  if(handle.hasParam("robot_info/max_speed_angular"))
  {
    handle.getParam("robot_info/max_speed_angular", max_speed_angular);
  }
  else
  {
    ROS_ERROR("Did not find robot_info/max_speed_angular rosparam");
    exit(1);
  }
  /*
   * Check for all costmap parameters!
   */
  /*if( handle.hasParam("costmap_node/costmap/width")     &&
      handle.hasParam("costmap_node/costmap/height")    &&
      handle.hasParam("costmap_node/costmap/origin_x")  &&
      handle.hasParam("costmap_node/costmap/origin_y") )
  {
    handle.getParam("costmap_node/costmap/width", costmap_width);
    handle.getParam("costmap_node/costmap/height", costmap_height);
    handle.getParam("costmap_node/costmap/origin_x", costmap_origin_x);
    handle.getParam("costmap_node/costmap/origin_y", costmap_origin_y);

    //ROS_INFO("Got costmap parameters. w: %f h: %f x: %f y: %f", costmap_width, costmap_height, costmap_origin_x, costmap_origin_y);

    float x_max = costmap_width + costmap_origin_x;
    float x_min = costmap_origin_x;
    float y_max = costmap_height + costmap_origin_y;
    float y_min = costmap_origin_y;
    //ROS_INFO("x_max: %f x_min: %f y_max: %f y_min: %f", x_max, x_min, y_max, y_min);
    
    std::vector<double> dof_min, dof_max;
    dof_min.push_back(x_min);
    dof_min.push_back(y_min);
    dof_max.push_back(x_max);
    dof_max.push_back(y_max);

    dof_min.push_back(-PI);
    dof_max.push_back(PI);

    initDOF(dof_min, dof_max); 
  }*/


  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> p_start;
    std::vector<float> p_goal;
    handle.getParam("robot_info/start", p_start);
    handle.getParam("robot_info/goal",  p_goal );
    initStartGoal(p_start, p_goal);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
    exit(1);
  }



  if(handle.hasParam("ramp/population_size")) 
  {
    handle.getParam("ramp/population_size", population_size);
    std::cout<<"\npopulation_size: "<<population_size;
  }

  
  if(handle.hasParam("ramp/sub_populations")) 
  {
    handle.getParam("ramp/sub_populations", sub_populations);
    std::cout<<"\nsub_populations: "<<sub_populations;
  }
  
  if(handle.hasParam("ramp/modifications")) 
  {
    handle.getParam("ramp/modifications", modifications);
    std::cout<<"\nmodifications: "<<modifications;
  }
  
  if(handle.hasParam("ramp/evaluations")) 
  {
    handle.getParam("ramp/evaluations", evaluations);
    std::cout<<"\nevaluations: "<<evaluations;
  }
  
  if(handle.hasParam("ramp/seed_population")) 
  {
    handle.getParam("ramp/seed_population", seedPopulation);
    std::cout<<"\nseed_population: "<<seedPopulation;
  }
  
  if(handle.hasParam("ramp/only_sensing"))
  {
    handle.getParam("ramp/only_sensing", only_sensing);
    std::cout<<"\nonly_sensing: "<<only_sensing;
  }

  if(handle.hasParam("ramp/moving_robot"))
  {
    handle.getParam("ramp/moving_robot", moving_robot);
    std::cout<<"\nmoving_robot: "<<moving_robot;
  }

  if(handle.hasParam("ramp/gens_before_control_cycle")) 
  {
    handle.getParam("ramp/gens_before_control_cycle", gensBeforeCC);
    std::cout<<"\ngens_before_control_cycle: "<<gensBeforeCC;
  }
  
  if(handle.hasParam("ramp/fixed_control_cycle_rate")) 
  {
    handle.getParam("ramp/fixed_control_cycle_rate", t_cc_rate);
    //ROS_INFO("t_cc_rate: %f", t_cc_rate);
  }
  
  if(handle.hasParam("ramp/sensing_cycle_rate")) 
  {
    handle.getParam("ramp/sensing_cycle_rate", t_sc_rate);
    //ROS_INFO("t_sc_rate: %f", t_sc_rate);
  }
  
  if(handle.hasParam("ramp/pop_traj_type")) 
  {
    handle.getParam("ramp/pop_traj_type", pop_type);
    //ROS_INFO("pop_type: %s", pop_type ? "Partial Bezier" : "All Straight");
    switch (pop_type) 
    {
      case 0:
        pt = HOLONOMIC;
        break;
      case 1:
        pt = HYBRID;
        break;
    }
  }
  
  if(handle.hasParam("ramp/error_reduction")) 
  {
    handle.getParam("ramp/error_reduction", errorReduction);
    //ROS_INFO("errorReduction: %s", errorReduction ? "True" : "False");
  }

  std::cout<<"\n------- Done loading parameters -------\n";
    std::cout<<"\n  ID: "<<id;
    std::cout<<"\n  Start: "<<start.toString();
    std::cout<<"\n  Goal: "<<goal.toString();
    std::cout<<"\n  Ranges: ";
    for(uint8_t i=0;i<ranges.size();i++) 
    {
      std::cout<<"\n  "<<i<<": "<<ranges.at(i).toString();
    }
  std::cout<<"\n---------------------------------------";
}



void pubStartGoalMarkers()
{
  ROS_INFO("In pubStartGoalMarkers");
  visualization_msgs::MarkerArray result;

  // Make Markers for both positions
  visualization_msgs::Marker start_marker, goal_marker;

  start_marker.header.stamp = ros::Time::now();
  goal_marker.header.stamp = ros::Time::now();
  start_marker.id = 10000;
  goal_marker.id = 10001;

  start_marker.header.frame_id = global_frame;
  goal_marker.header.frame_id = global_frame;

  start_marker.ns = "basic_shapes";
  goal_marker.ns = "basic_shapes";

  start_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.type = visualization_msgs::Marker::SPHERE;

  start_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.action = visualization_msgs::Marker::ADD;
  
  // Set positions
  start_marker.pose.position.x = start.msg_.positions[0];
  start_marker.pose.position.y = start.msg_.positions[1];
  start_marker.pose.position.z = 0.01;
  
  goal_marker.pose.position.x = goal.msg_.positions[0];
  goal_marker.pose.position.y = goal.msg_.positions[1];
  goal_marker.pose.position.z = 0.01;

  // Set orientations
  start_marker.pose.orientation.x = 0.0;
  start_marker.pose.orientation.y = 0.0;
  start_marker.pose.orientation.z = 0.0;
  start_marker.pose.orientation.w = 1.0;
 
  goal_marker.pose.orientation.x = 0.0;
  goal_marker.pose.orientation.y = 0.0;
  goal_marker.pose.orientation.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;

  // Set radii
  start_marker.scale.x = 0.5;
  start_marker.scale.y = 0.5;
  start_marker.scale.z = 0.1;
 
  goal_marker.scale.x = 0.5;
  goal_marker.scale.y = 0.5;
  goal_marker.scale.z = 0.1;

  // Set colors
  start_marker.color.r = 1;
  start_marker.color.g = 0;
  start_marker.color.b = 0;
  start_marker.color.a = 1;
 
  goal_marker.color.r = 0;
  goal_marker.color.g = 0;
  goal_marker.color.b = 1;
  goal_marker.color.a = 1;

  // Set lifetimes
  start_marker.lifetime = ros::Duration(10.0);
  goal_marker.lifetime = ros::Duration(10.0);

  // Create marker array and publish
  result.markers.push_back(start_marker);
  result.markers.push_back(goal_marker);

  while(ros::ok() && pub_rviz.getNumSubscribers() == 0) {}
  //ROS_INFO("# of subscribers: %i", (int)pub_rviz.getNumSubscribers());

  pub_rviz.publish(result);
  pub_rviz.publish(result);
  
  ROS_INFO("Exiting pubStartGoalMarkers");
}

void deltaParasCallback(const ramp_msgs::ParameterUpdates::ConstPtr &delta_paras_ptr) {
	return;
}

void killNodes() {
  int a = 0; // remove warning in compling
  a = system("rosnode kill /move_obstacles");
  a = system("rosnode kill /ramp_control");
  a = system("rosnode kill /ramp_sensing");
  a = system("rosnode kill /trajectory_evaluation");
  a = system("rosnode kill /path_modification");
  a = system("rosnode kill /trajectory_generator");
  a = system("rosnode kill /pub_map_equal_odom");
}

bool doNothing(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res) {
  return true;
}

int main(int argc, char** argv) 
{
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  bool use_sim_time;
  ros::param::param("/use_sim_time", use_sim_time, false);
  ros::ServiceClient gazebo_srv_client = handle.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  if (use_sim_time) {
    std_srvs::Empty gazebo_srv;
    //// reset the robot in gazebo
    if (gazebo_srv_client.call(gazebo_srv)) {
      printf("The robot in gazebo has been reset!\n");
    } else {
      printf("Please launch gazebo if you want simulation.\n");
    }
  }

  ///// reset odom.
  ros::Publisher pub_reset_odom = handle.advertise<std_msgs::Empty>("mobile_base/commands/reset_odometry", 10);
  std_msgs::Empty reset_odom_msg;
  ros::Duration one_sec(1.0);
  one_sec.sleep();
  pub_reset_odom.publish(reset_odom_msg);

  ros::param::set("ramp/cc_started", false);
  std::cout<<"\nHandle namespace: "<<handle.getNamespace();
  
  // Load ros parameters
  loadParameters(handle);

  Planner my_planner;

  ros::Publisher pub_exe_time_ = handle.advertise<std_msgs::Float64>("execution_time", 10);
  ros::Publisher pub_ramp_collection_exe_time_ = handle.advertise<std_msgs::Float64>("ramp_collection_exe_time", 1, true);
  ros::Publisher pub_ramp_collection_ramp_ob_one_run_ = handle.advertise<ramp_msgs::RampObservationOneRunning>("ramp_collection_ramp_ob_one_run", 1, true);
  
  // Use updateCallbackPose if the msg type is PoseWithCovarianceStamped
  if(update_topic != "odom")
  {
    ros::Subscriber sub_updatePose_ = handle.subscribe(update_topic, 1, &Planner::updateCbPose, &my_planner);
  }
  ros::Subscriber sub_updateVel_ = handle.subscribe("update", 1, &Planner::updateCbControlNode, &my_planner);
  ros::Subscriber sub_sc_ = handle.subscribe("obstacles", 1, &Planner::sensingCycleCallback, &my_planner);
  ros::Subscriber sub_delta_paras_ = handle.subscribe("delta_paras", 10, &deltaParasCallback);
  
  ros::ServiceServer env_service = handle.advertiseService(ENV_SRV_NAME, &Planner::envSrvCallback, &my_planner);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  /*
   * Get the transform from map to the Planner's global_frame
   */

  // Sleep to wait for tf data
  // Using the sleep in waitForTransform never works
  ros::Duration d(2.0);
  d.sleep();

  // Wait for Transform from map to the planner's global_frame
  // Since this is used to transform poses from amcl, how can I generalize this from "map"?
  if(global_frame != "map")
  {
    tf::TransformListener listener_;
    if(listener_.waitForTransform(global_frame, "map", ros::Time(0), d))
    {
      listener_.lookupTransform(global_frame, "map", ros::Time(0), my_planner.tf_global_costmap_);
      //ROS_INFO("Costmap tf: translate: (%f, %f) rotation: %f", my_planner.tf_global_costmap_.getOrigin().getX(), my_planner.tf_global_costmap_.getOrigin().getX(), my_planner.tf_global_costmap_.getRotation().getAngle());
    }
    else
    {
      //ROS_INFO("Frame \"map\" does not exist");
    }
  }

  d.sleep();

  /*
   * Get the transform from odom to global frame
   */
  tf::TransformListener listen;
  listen.waitForTransform(global_frame, "odom", ros::Time(0), ros::Duration(2.0));
  listen.lookupTransform(global_frame, "odom", ros::Time(0), my_planner.tf_global_odom_);
  //ROS_INFO("Odom tf: translate: (%f, %f) rotation: %f", my_planner.tf_global_odom_.getOrigin().getX(), my_planner.tf_global_odom_.getOrigin().getX(), my_planner.tf_global_odom_.getRotation().getAngle());
  
  my_planner.tf_global_odom_rot_ = my_planner.tf_global_odom_;
  my_planner.tf_global_odom_rot_.setOrigin( tf::Vector3(0, 0, 0) );
  
  /*
   * Check that the start and goal are within specified ranges
   */
  for(int i=0;i<ranges.size();i++)
  {
    if( start.msg_.positions[i] < ranges[i].msg_.min || start.msg_.positions[i] > ranges[i].msg_.max ||
        goal.msg_.positions[i] < ranges[i].msg_.min || goal.msg_.positions[i] > ranges[i].msg_.max )
    {
      ROS_ERROR("Either the Start or goal position is not within DOF ranges, exiting ramp_planner");
      exit(1);
    }
  }

  /*
   * All parameters are loaded
   */
 
  // Initialize the planner
  my_planner.init(id, handle, start, goal, ranges, max_speed_linear, max_speed_angular, population_size, radius, sub_populations,
                  global_frame, update_topic, pt, gensBeforeCC, t_sc_rate, t_cc_rate, only_sensing, moving_robot, errorReduction); 
  my_planner.modifications_   = modifications;
  my_planner.evaluations_     = evaluations;
  my_planner.seedPopulation_  = seedPopulation;

  std::cout<<"\nStart: "<<my_planner.start_.toString();
  std::cout<<"\nGoal: "<<my_planner.goal_.toString();

  pubStartGoalMarkers();
  ROS_INFO("pubStartGoalMarkers complete");
  ////ROS_INFO("Done with pubStartGoalMarkers");
 
  //// msg published anywhere in this file must be between start rosbag and kill rosbag
  //   robot_radius, latch = true
  ros::Publisher pub_robot_radius_ = handle.advertise<std_msgs::Float64>("ramp_collection_robot_radius", 1, true);
  std_msgs::Float64 msg_1; msg_1.data = radius; pub_robot_radius_.publish(msg_1);
  //   population_size
  ros::Publisher pub_population_size_ = handle.advertise<std_msgs::Int64>("ramp_collection_population_size", 1, true);
  std_msgs::Int64 msg_2; msg_2.data = population_size; pub_population_size_.publish(msg_2);
  //   sub_populations
  ros::Publisher pub_sub_populations_ = handle.advertise<std_msgs::Bool>("ramp_collection_sub_populations", 1, true);
  std_msgs::Bool msg_3; msg_3.data = sub_populations; pub_sub_populations_.publish(msg_3);
  //   gens_before_control_cycle
  ros::Publisher pub_gens_before_control_cycle_ = handle.advertise<std_msgs::Int64>("ramp_collection_gens_before_control_cycle", 1, true);
  std_msgs::Int64 msg_4; msg_4.data = gensBeforeCC; pub_gens_before_control_cycle_.publish(msg_4);
  //   pop_traj_type
  ros::Publisher pub_pop_traj_type_ = handle.advertise<std_msgs::Int64>("ramp_collection_pop_traj_type", 1, true);
  std_msgs::Int64 msg_5; msg_5.data = pop_type; pub_pop_traj_type_.publish(msg_5);
  //   fixed_control_cycle_rate
  ros::Publisher pub_fixed_control_cycle_rate_ = handle.advertise<std_msgs::Float64>("ramp_collection_fixed_control_cycle_rate", 1, true);
  std_msgs::Float64 msg_6; msg_6.data = t_cc_rate; pub_fixed_control_cycle_rate_.publish(msg_6);
  //   sensing_cycle_rate
  ros::Publisher pub_sensing_cycle_rate_ = handle.advertise<std_msgs::Float64>("ramp_collection_sensing_cycle_rate", 1, true);
  std_msgs::Float64 msg_7; msg_7.data = t_sc_rate; pub_sensing_cycle_rate_.publish(msg_7);
  //   x_range (x_min and x_max)
  ros::Publisher pub_x_range_ = handle.advertise<ramp_msgs::Range>("ramp_collection_x_range", 1, true);
  ramp_msgs::Range msg_8; msg_8 = ranges[0].msg_; pub_x_range_.publish(msg_8);
  //   y_range
  ros::Publisher pub_y_range_ = handle.advertise<ramp_msgs::Range>("ramp_collection_y_range", 1, true);
  ramp_msgs::Range msg_9; msg_9 = ranges[1].msg_; pub_y_range_.publish(msg_9);
  //   theta_range
  ros::Publisher pub_theta_range_ = handle.advertise<ramp_msgs::Range>("ramp_collection_theta_range", 1, true);
  ramp_msgs::Range msg_10; msg_10 = ranges[2].msg_; pub_theta_range_.publish(msg_10);
  //   max_speed_linear
  ros::Publisher pub_max_speed_linear_ = handle.advertise<std_msgs::Float64>("ramp_collection_max_speed_linear", 1, true);
  std_msgs::Float64 msg_11; msg_11.data = max_speed_linear; pub_max_speed_linear_.publish(msg_11);
  //   max_speed_angular
  ros::Publisher pub_max_speed_angular_ = handle.advertise<std_msgs::Float64>("ramp_collection_max_speed_angular", 1, true);
  std_msgs::Float64 msg_12; msg_12.data = max_speed_angular; pub_max_speed_angular_.publish(msg_12);
  //   start
  ros::Publisher pub_start_motion_state_ = handle.advertise<ramp_msgs::MotionState>("ramp_collection_start_motion_state", 1, true);
  ramp_msgs::MotionState msg_13; msg_13 = start.msg_; pub_start_motion_state_.publish(msg_13);
  //   goal
  ros::Publisher pub_goal_motion_state_ = handle.advertise<ramp_msgs::MotionState>("ramp_collection_goal_motion_state", 1, true);
  ramp_msgs::MotionState msg_14; msg_14 = goal.msg_; pub_goal_motion_state_.publish(msg_14);
  
  //// prepare to tell others that I am ready
  ros::Publisher pub_set_env_rdy_true_ = handle.advertise<std_msgs::Empty>("set_env_ready_true", 1, true);// latch = true
  std_msgs::Empty empty_msg;

  /******* Start the planner *******/
  if(use_start_param) // TODO: use triggered srv
  {
    ros::Rate r_wait_start(10); // 10Hz
    start_planner = false;
    handle.setParam("ramp/start_planner", false);
    ROS_INFO("Waiting for param ramp/start_planner to be true");
    pub_set_env_rdy_true_.publish(empty_msg); // tell others that I am ready
    ros::ServiceServer env_ready_srv = handle.advertiseService("env_ready_srv", doNothing); // tell others that I am ready
                                                                                            // others just need to check whether
                                                                                            // this srv exists, no need to call
                                                                                            // this srv actually.
    while(ros::ok() && !start_planner)
    {
      handle.param("ramp/start_planner", start_planner, false);
      r_wait_start.sleep();
      ros::spinOnce();
    }
    env_ready_srv.shutdown(); // shutdown the srv once I am asked to run
  }
  else
  {
    std::cout<<"\nPress Enter to start the planner\n";
    pub_set_env_rdy_true_.publish(empty_msg); // tell others that I am ready
    ros::ServiceServer env_ready_srv = handle.advertiseService("env_ready_srv", doNothing); // tell others that I am ready
    std::cin.get();
    env_ready_srv.shutdown(); // shutdown the srv once I am asked to run
  }

  ROS_INFO("Starting Planner!");
  
  my_planner.go(handle);

  //// Publish ramp observation after one running.
  //   Note that ramp observation after one step is calculated in environment interface, not here.
  pub_ramp_collection_ramp_ob_one_run_.publish(my_planner.obser_one_run);

  //// publish execution time
  std_msgs::Float64 et_msg;
  et_msg.data = my_planner.d_runtime_.toSec(); // seconds
  pub_exe_time_.publish(et_msg);
  pub_ramp_collection_exe_time_.publish(et_msg);

  //// publish the number of best trajectory switches during the execution
  ros::Publisher pub_nb_best_traj_switches_ = handle.advertise<std_msgs::Int64>("ramp_collection_nb_best_traj_switches",
                                                                                1, true);
  std_msgs::Int64 msg_nb_best_t_switch;
  msg_nb_best_t_switch.data = my_planner.h_control_->nb_best_traj_switches;
  pub_nb_best_traj_switches_.publish(msg_nb_best_t_switch);

  //// publish offline reward
  std_msgs::Float64 reward;
  reward.data = my_planner.population_.getBest().reward();
  ros::Publisher r_pub = handle.advertise<std_msgs::Float64>("ramp_collection_reward_offline", 1, true);
  r_pub.publish(reward);

  //// kill all other nodes in the ros launch file
  killNodes();
 
  //****MotionState exp_results = my_planner.findAverageDiff();
  //****std::cout<<"\n\nAverage Difference: "<<exp_results.toString();
  
  
  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}

