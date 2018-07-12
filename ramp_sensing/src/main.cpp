#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/ObstacleList.h"
#include "obstacle.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "circle_packer.h"
#include "circle_filter.h"
#include <visualization_msgs/MarkerArray.h>
#include "utility.h"


#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/pdf/analyticconditionalgaussian.h>


bool use_static_map = false;
bool gotPersistent = false;
bool use_odom_topics = false;
bool use_hilbert_map = false;
bool planner_started = false;



bool cropMap = false;
bool remove_outside_fov;
double rvizLifetime = 0.15;

ramp_msgs::MotionState robotState;
ramp_msgs::MotionState initialOdomState;
bool initOdom = false;


double fovAngle;
std::vector<double> viewMinMax;

std::vector<CircleOb*> cir_obs;
std::vector<Circle> cirs_pos;

std::vector<CircleGroup> cirGroups;
std::vector< std::vector<Circle> > cirs;
visualization_msgs::Marker polygonLines;
std::vector<visualization_msgs::Marker> pLines;
std::vector<visualization_msgs::Marker> cLines;

nav_msgs::OccupancyGrid global_costmap;
nav_msgs::OccupancyGrid staticMap;

Utility util;
double rate;
ros::Publisher pub_obj, pub_rviz, pub_cons_costmap, pub_half_costmap, pub_global_costmap;
std::vector<Obstacle> obs;
ramp_msgs::ObstacleList list;
ramp_msgs::ObstacleList staticObsList;
ramp_msgs::ObstacleList dynamicObsList;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;
nav_msgs::OccupancyGrid global_grid;

double radius;
std::vector<double> dof_min;
std::vector<double> dof_max;
std::vector<ramp_msgs::Range> ranges;

double min_arrow_len = 0.25;

std::vector<tf::Transform> ob_tfs;

std::vector<Circle> prev_cirs;

// prev_velocities[cycle][circle]
std::vector< std::vector<Velocity> > prev_velocities;

ros::Time t_last_costmap;

int num_costmaps=0;


std::vector<nav_msgs::OccupancyGrid> prev_grids;

std::vector<CircleGroup> prev_valid_cirs;


// Vector to store velocities to report data after execution
std::vector<Velocity> predicted_velocities;

ros::Timer timer_markers;

double coll_radius = 0.25;

std::vector<double> d_avg_values;

double dist_threshold = 0.5;
double radius_threshold = 0.5;

int num_costmaps_accumulate = 5;
int num_velocity_count      = 10;
int num_theta_count         = 1;
int num_costmap_freq_theta  = 5;

double static_v_threshold   = 0.3;
int    ob_not_moving_count  = 10;

// Obstacles that we want to save the circlegroup for
std::vector<CircleGroup> largeObs;

std::vector<CircleGroup> staticObs;

double initial_theta        = PI;
              

std::string robot_base_frame, global_frame;
tf::StampedTransform tf_base_to_global;
  
std::vector<Attachment> attachs;

int populationSize;

std::vector<double> durs;


/*********************************
 * Variables for BFL
 *********************************/


/*
 * Linear System
 */
BFL::LinearAnalyticConditionalGaussian* sys_pdf=0;
BFL::LinearAnalyticSystemModelGaussianUncertainty* sys_model=0;

int STATE_SIZE=4;
double MU_SYSTEM_NOISE_X = 0.1;
double MU_SYSTEM_NOISE_Y = 0.1;
double SIGMA_SYSTEM_NOISE_X = 0.1;
double SIGMA_SYSTEM_NOISE_Y = 0.1;

/*
 * Measurement model
 */
//MatrixWrapper::Matrix* H=0;
BFL::LinearAnalyticConditionalGaussian* meas_pdf = 0;
BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model = 0;
double MU_MEAS_NOISE    = 0.0000001;
double SIGMA_MEAS_NOISE = 0.0000001;

// Input vector
MatrixWrapper::ColumnVector u(STATE_SIZE);

/*
 * Prior distribution
 */
BFL::Gaussian* prior = 0;
double PRIOR_MU_X = 328;
double PRIOR_MU_Y = 211;
double PRIOR_MU_VX = 0.01;
double PRIOR_MU_VY = 0.01;
double PRIOR_MU_AX = 0.01;
double PRIOR_MU_AY = 0.01;
double PRIOR_COV_X = 0.01;
double PRIOR_COV_Y = 0.01;
double PRIOR_COV_VX = 0.01;
double PRIOR_COV_VY = 0.01;
double PRIOR_COV_AX = 0.01;
double PRIOR_COV_AY = 0.01;


BFL::Pdf<MatrixWrapper::ColumnVector>* posterior;


int costmap_width, costmap_height;
float costmap_origin_x, costmap_origin_y, costmap_res;

std::vector<double> start;

// Initializes a vector of Ranges that the Planner is initialized with
// Must be called AFTER radius is set
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  // DOF are [x, y, theta], we don't want theta so we do -1
  for(unsigned int i=0;i<dof_min.size()-1;i++) 
  {
    ramp_msgs::Range temp;
    temp.min = dof_min.at(i);
    temp.max = dof_max.at(i);
    /*if(i == 0 || i == 1)
    {
      temp.min += radius;
      temp.max -= radius;
    }*/
    ranges.push_back(temp); 
  }

} // End initDOF


void loadParameters(const ros::NodeHandle& handle)
{
  /*
   * Check for all costmap parameters!
   */
  if( handle.hasParam("costmap_node/costmap/width")     &&
      handle.hasParam("costmap_node/costmap/height")    &&
      handle.hasParam("costmap_node/costmap/origin_x")  &&
      handle.hasParam("costmap_node/costmap/origin_y") )
  {
    handle.getParam("costmap_node/costmap/width", costmap_width);
    handle.getParam("costmap_node/costmap/height", costmap_height);
    handle.getParam("costmap_node/costmap/origin_x", costmap_origin_x);
    handle.getParam("costmap_node/costmap/origin_y", costmap_origin_y);
    handle.param("costmap_node/costmap/resolution", costmap_res, 0.05f);

    //////////ROS_INFO("Got costmap parameters. w: %i h: %i x: %f y: %f res: %f", costmap_width, costmap_height, costmap_origin_x, costmap_origin_y, costmap_res);
  }

  // Get the radius of the robot
  if(handle.hasParam("robot_info/radius")) 
  {
    handle.getParam("robot_info/radius", radius);
  }
  else 
  {
    ////////ROS_ERROR("Did not find parameter robot_info/radius");
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
    ////////ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }

  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start"))
  {
    handle.getParam("robot_info/start", start);
  }
  else 
  {
    //ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
    exit(1);
  }


  if(handle.hasParam("/ramp/global_frame"))
  {
    handle.getParam("/ramp/global_frame", global_frame);
    ////////ROS_INFO("global_frame: %s", global_frame.c_str());
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /ramp/global_frame");
  }

  if(handle.hasParam("/costmap_node/costmap/robot_base_frame"))
  {
    handle.getParam("/costmap_node/costmap/robot_base_frame", robot_base_frame);
    ////////ROS_INFO("robot_base_frame: %s", robot_base_frame.c_str());
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /costmap_node/costmap/robot_base_frame");
    robot_base_frame = "map";
  }

  if(handle.hasParam("/ramp/population_size"))
  {
    handle.getParam("/ramp/population_size", populationSize);
    ////////ROS_INFO("populationSize: %i", populationSize);
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /ramp/population_size");
  }
  
  // Get the radius of the robot
  if(handle.hasParam("ramp/sensing_cycle_rate")) 
  {
    handle.getParam("ramp/sensing_cycle_rate", rate);
  }
  else 
  {
    ////////ROS_ERROR("Did not find parameter robot_info/radius");
  }

  if(handle.hasParam("/ramp/field_of_view_angle"))
  {
    handle.getParam("/ramp/field_of_view_angle", fovAngle);
    ////////ROS_INFO("fovAngle: %f", fovAngle);
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /ramp/field_of_view_angle");
  }

  if(handle.hasParam("/ramp/remove_outside_fov"))
  {
    handle.getParam("/ramp/remove_outside_fov", remove_outside_fov);
    ////////ROS_INFO("remove_outside_fov", remove_outside_fov);
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /ramp/remove_outside_fov");
  }
  
  if(handle.hasParam("/ramp/use_static_map"))
  {
    handle.getParam("/ramp/use_static_map", use_static_map);
  }
  else
  {
    ////////ROS_ERROR("Did not find rosparam /ramp/use_persistent_grid");
  }


  if(handle.hasParam("ramp/use_odom_topics"))
  {
    handle.getParam("ramp/use_odom_topics", use_odom_topics);
  }

  if(handle.hasParam("ramp/use_hilbert_map"))
  {
    handle.getParam("ramp/use_hilbert_map", use_hilbert_map);
  }


  
}



void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    //////////ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      //////////ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        //////////ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(std::stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      //////////ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(std::stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}


void init_measurement_model()
{
  //////////ROS_INFO("In init_measurement_model");

  MatrixWrapper::Matrix H(STATE_SIZE,STATE_SIZE);
  H = 0.0;

  // Set x and y
  H(1,1) = 1;
  H(2,2) = 1;

  //////////ROS_INFO("Setting mu");

  BFL::ColumnVector meas_noise_mu(STATE_SIZE);
  meas_noise_mu(1) = MU_MEAS_NOISE;
  meas_noise_mu(2) = MU_MEAS_NOISE;
  meas_noise_mu(3) = 0;
  meas_noise_mu(4) = 0;

  //////////ROS_INFO("Setting cov");

  MatrixWrapper::SymmetricMatrix meas_noise_cov(STATE_SIZE);
  meas_noise_cov = 0.0;
  for(int i=1;i<=STATE_SIZE;i++)
  {
    meas_noise_cov(i,i) = SIGMA_MEAS_NOISE;
  }

  //////////ROS_INFO("Setting measurement_uncertainty");

  // Make the Gaussian
  BFL::Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);

  //////////ROS_INFO("Setting meas_pdf");

  // Make the pdf
  meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty);
}


void init_prior_model()
{
  //////////ROS_INFO("In init_prior_model");

  // Build prior distribution
  BFL::ColumnVector prior_mu(STATE_SIZE);
  prior_mu(1) = PRIOR_MU_X;
  prior_mu(2) = PRIOR_MU_Y;
  prior_mu(3) = PRIOR_MU_VX;
  prior_mu(4) = PRIOR_MU_VY;

  MatrixWrapper::SymmetricMatrix prior_cov(STATE_SIZE);
  prior_cov = 0.0;
  prior_cov(1,1) = PRIOR_COV_X;
  prior_cov(2,2) = PRIOR_COV_Y;
  prior_cov(3,3) = PRIOR_COV_VX;
  prior_cov(4,4) = PRIOR_COV_VY;

  prior = new BFL::Gaussian(prior_mu, prior_cov);
}


/*
 * Initialize a linear system model
 */
void init_linear_system_model()
{
  //////////ROS_INFO("In init_linear_system_model");

  MatrixWrapper::ColumnVector sys_noise_Mu(STATE_SIZE);
  for(int i=1;i<=STATE_SIZE;i++)
  {
    sys_noise_Mu(i) = MU_SYSTEM_NOISE_X;
  }


  MatrixWrapper::SymmetricMatrix sys_noise_Cov(STATE_SIZE);
  sys_noise_Cov = 0.0;
  for(int i=1;i<=STATE_SIZE;i++)
  {
    sys_noise_Cov(i,i) = SIGMA_SYSTEM_NOISE_X;
  }

  /*
   * Need two matrices, A and B
   * A is multiplied by the current state
   * B is multiplied by the input
   * X_(t+1) = Ax_t + Bu_t
   */
  MatrixWrapper::Matrix A(4,4);
  A = 0;
  A(1,1) = 1;
  A(2,2) = 1;
  A(3,3) = 1;
  A(4,4) = 1;
  MatrixWrapper::Matrix B(4,4);
  B = 0;
  B(1,1) = 1;
  B(2,2) = 1;
  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  //////////ROS_INFO("Initializing the linear system uncertainty"); 

  BFL::Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

  sys_pdf = new BFL::LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
}



/** Get the other robot's current odometry information and update the obstacle info */
void updateOtherRobotCb(const nav_msgs::Odometry::ConstPtr& o, const std::string& topic) 
{
  //ROS_INFO("In updateOtherRobotCb");
  //ROS_INFO("topic: %s", topic.c_str());
  int index = topic_index_map[topic];
  //ROS_INFO("index: %i", index);
  
  //ROS_INFO("obs.size(): %i", (int)obs.size());

  if(obs.size() < index+1)
  {
    //ROS_INFO("In if obs.size() < index");
    Obstacle temp;
    temp.update(*o);
    obs.push_back(temp);
    dynamicObsList.obstacles.push_back(temp.msg_);
  }
  else
  {
    //ROS_INFO("In else");
    obs.at(index).update(*o);
    dynamicObsList.obstacles.at(index) = obs.at(index).msg_;
  }

  //ROS_INFO("dynamicObsList.obstacles.size(): %i", (int)dynamicObsList.obstacles.size());
  //ROS_INFO("Exiting updateOtherRobotCb");
} //End updateOtherRobotCb



void addHardCodedStaticObs()
{
  //ROS_INFO("In addHardCodedStaticObs");
  //dynamicObsList.obstacles.clear();

  Obstacle oa;
  oa.msg_.ob_ms.positions.push_back(2.0);
  oa.msg_.ob_ms.positions.push_back(2.0);
  oa.msg_.ob_ms.positions.push_back(0);
  oa.msg_.cirGroup.fitCir.center.x = oa.msg_.ob_ms.positions[0];
  oa.msg_.cirGroup.fitCir.center.y = oa.msg_.ob_ms.positions[1];
  oa.msg_.cirGroup.fitCir.radius = 0.25;
  oa.msg_.cirGroup.packedCirs.push_back(oa.msg_.cirGroup.fitCir);
  oa.msg_.ob_ms.velocities.push_back(0);
  oa.msg_.ob_ms.velocities.push_back(0);
  oa.msg_.ob_ms.velocities.push_back(0);


  Obstacle ob;
  ob.msg_.ob_ms.positions.push_back(2.3);
  ob.msg_.ob_ms.positions.push_back(1.7);
  ob.msg_.ob_ms.positions.push_back(0);
  ob.msg_.cirGroup.fitCir.center.x = ob.msg_.ob_ms.positions[0];
  ob.msg_.cirGroup.fitCir.center.y = ob.msg_.ob_ms.positions[1];
  ob.msg_.cirGroup.fitCir.radius = 0.25;
  ob.msg_.cirGroup.packedCirs.push_back(ob.msg_.cirGroup.fitCir);
  ob.msg_.ob_ms.velocities.push_back(0);
  ob.msg_.ob_ms.velocities.push_back(0);
  ob.msg_.ob_ms.velocities.push_back(0);


  Obstacle oc;
  oc.msg_.ob_ms.positions.push_back(2.5);
  oc.msg_.ob_ms.positions.push_back(1.35);
  oc.msg_.ob_ms.positions.push_back(0);
  oc.msg_.cirGroup.fitCir.center.x = oc.msg_.ob_ms.positions[0];
  oc.msg_.cirGroup.fitCir.center.y = oc.msg_.ob_ms.positions[1];
  oc.msg_.cirGroup.fitCir.radius = 0.22;
  oc.msg_.cirGroup.packedCirs.push_back(oc.msg_.cirGroup.fitCir);
  oc.msg_.ob_ms.velocities.push_back(0);
  oc.msg_.ob_ms.velocities.push_back(0);
  oc.msg_.ob_ms.velocities.push_back(0);

  Obstacle od;
  od.msg_.ob_ms.positions.push_back(0.8);
  od.msg_.ob_ms.positions.push_back(2.8);
  od.msg_.ob_ms.positions.push_back(0);
  od.msg_.cirGroup.fitCir.center.x = od.msg_.ob_ms.positions[0];
  od.msg_.cirGroup.fitCir.center.y = od.msg_.ob_ms.positions[1];
  od.msg_.cirGroup.fitCir.radius = 0.2;
  od.msg_.cirGroup.packedCirs.push_back(od.msg_.cirGroup.fitCir);
  od.msg_.ob_ms.velocities.push_back(0);
  od.msg_.ob_ms.velocities.push_back(0);
  od.msg_.ob_ms.velocities.push_back(0);

  Obstacle oe;
  oe.msg_.ob_ms.positions.push_back(1.2);
  oe.msg_.ob_ms.positions.push_back(2.8);
  oe.msg_.ob_ms.positions.push_back(0);
  oe.msg_.cirGroup.fitCir.center.x = oe.msg_.ob_ms.positions[0];
  oe.msg_.cirGroup.fitCir.center.y = oe.msg_.ob_ms.positions[1];
  oe.msg_.cirGroup.fitCir.radius = 0.2;
  oe.msg_.cirGroup.packedCirs.push_back(oe.msg_.cirGroup.fitCir);
  oe.msg_.ob_ms.velocities.push_back(0);
  oe.msg_.ob_ms.velocities.push_back(0);
  oe.msg_.ob_ms.velocities.push_back(0);

  /*Obstacle of;
  of.msg_.ob_ms.positions.push_back(2.0);
  of.msg_.ob_ms.positions.push_back(3.25);
  of.msg_.ob_ms.positions.push_back(0);
  of.msg_.cirGroup.fitCir.center.x = of.msg_.ob_ms.positions[0];
  of.msg_.cirGroup.fitCir.center.y = of.msg_.ob_ms.positions[1];
  of.msg_.cirGroup.fitCir.radius = 0.25;
  of.msg_.cirGroup.packedCirs.push_back(of.msg_.cirGroup.fitCir);
  of.msg_.ob_ms.velocities.push_back(0);
  of.msg_.ob_ms.velocities.push_back(0);
  of.msg_.ob_ms.velocities.push_back(0);

  Obstacle og;
  og.msg_.ob_ms.positions.push_back(3.2);
  og.msg_.ob_ms.positions.push_back(0.33);
  og.msg_.ob_ms.positions.push_back(0);
  og.msg_.cirGroup.fitCir.center.x = og.msg_.ob_ms.positions[0];
  og.msg_.cirGroup.fitCir.center.y = og.msg_.ob_ms.positions[1];
  og.msg_.cirGroup.fitCir.radius = 0.33;
  og.msg_.cirGroup.packedCirs.push_back(og.msg_.cirGroup.fitCir);
  og.msg_.ob_ms.velocities.push_back(0);
  og.msg_.ob_ms.velocities.push_back(0);
  og.msg_.ob_ms.velocities.push_back(0);*/

  if(dynamicObsList.obstacles.size() > 6)
  {
    dynamicObsList.obstacles[2] = oa.msg_;
    dynamicObsList.obstacles[3] = ob.msg_;
    dynamicObsList.obstacles[4] = oc.msg_;
    dynamicObsList.obstacles[5] = od.msg_;
    dynamicObsList.obstacles[6] = oe.msg_;
  }
  else
  {
    dynamicObsList.obstacles.push_back(oa.msg_);
    dynamicObsList.obstacles.push_back(ob.msg_);
    dynamicObsList.obstacles.push_back(oc.msg_);
    dynamicObsList.obstacles.push_back(od.msg_);
    dynamicObsList.obstacles.push_back(oe.msg_);
  }
  //dynamicObsList.obstacles.push_back(of.msg_);
  //dynamicObsList.obstacles.push_back(og.msg_);
  //ROS_INFO("Exiting addHardCodedStaticObs, dynamicObsList.obstacles.size(): %i", (int)dynamicObsList.obstacles.size());
}


std::vector<visualization_msgs::Marker> convertObsToMarkers(const ramp_msgs::ObstacleList& obList, int lastId)
{
  ROS_INFO("In convertObsToMarkers");
  std::vector<visualization_msgs::Marker> result;
  //ROS_INFO("dynamicObsList.obstacles.size(): %i", (int)dynamicObsList.obstacles.size());
  
  if(use_odom_topics || obList.obstacles.size() > 0 || (global_grid.info.width != 0 && global_grid.info.height != 0))
  {
    ROS_INFO("obList.obstacles.size(): %i", (int)obList.obstacles.size());
    for(int i=0;i<obList.obstacles.size();i++)
    {
      ROS_INFO("Obstacle %i, (x,y): (%f,%f)", i, obList.obstacles[i].ob_ms.positions[0], obList.obstacles[i].ob_ms.positions[1]);
    }

    //int iStop = (use_odom_topics) ? obList.obstacles.size() : cir_obs.size();
    int iStop = obList.obstacles.size();
    //ROS_INFO("iStop: %i obList.obstacles.size(): %i cir_obs.size(): %i", iStop, (int)obList.obstacles.size(), (int)cir_obs.size());
    // For each CircleOb, make a Marker
    for(int i=0;i<iStop;i++)
    {
      //ROS_INFO("i: %i", i);
      visualization_msgs::Marker cirMarker;
      cirMarker.header.stamp = ros::Time::now();
      cirMarker.header.frame_id = global_frame;
      cirMarker.ns = "basic_shapes";
      cirMarker.id = (populationSize * i) + lastId;
      //ROS_INFO("Bounding Circle id: %i popSize: %i i: %i", cirMarker.id, populationSize, i);
        
      cirMarker.type = visualization_msgs::Marker::SPHERE;
      cirMarker.action = visualization_msgs::Marker::ADD;

      // Set x and y
      double x = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.fitCir.center.x : cir_obs[i]->cirGroup.fitCir.center.x;
      double y = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.fitCir.center.y : cir_obs[i]->cirGroup.fitCir.center.y;
        
      cirMarker.pose.position.x = x;
      cirMarker.pose.position.y = y;
      cirMarker.pose.position.z = 0;
      cirMarker.pose.orientation.x = 0.0;
      cirMarker.pose.orientation.y = 0.0;
      cirMarker.pose.orientation.z = 0.0;
      cirMarker.pose.orientation.w = 1.0;

      //double radius = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.fitCir.radius : cir_obs[i]->cirGroup.fitCir.radius;
      double radius = obList.obstacles[i].cirGroup.fitCir.radius;
      ROS_INFO("Fit circle i: %i x: %f y: %f r: %f", i, x, y, radius);
        
      // scale values are the diameter so use the radius*2
      cirMarker.scale.x = radius*2.00f;
      cirMarker.scale.y = radius*2.00f;
      cirMarker.scale.z = 0.1;
      cirMarker.color.r = 0;
      cirMarker.color.g = 1;
      cirMarker.color.b = 0;
      cirMarker.color.a = 0.25;
      cirMarker.lifetime = ros::Duration(rvizLifetime);

      result.push_back(cirMarker);

      int jStop = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.packedCirs.size() : cir_obs[i]->cirGroup.packedCirs.size();
      ROS_INFO("jStop: %i obList.obstacles[i].cirGroup.packedCirs.size(): %i", jStop, (int)obList.obstacles[i].cirGroup.packedCirs.size());
      for(int j=0;j<jStop;j++)
      {
        //ROS_INFO("j: %i", j);
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = global_frame;
        marker.ns = "basic_shapes";

        // FIX THIS B/C THERE ARE SOME OVERLAPS
        marker.id = (20000 + (populationSize * i) + j) + lastId;
        ROS_INFO("Packed cir id: %i popSize: %i i: %i j: %i", marker.id, populationSize, i, j);
        
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set x and y
        double x = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.packedCirs[j].center.x : cir_obs[i]->cirGroup.packedCirs[j].center.x;
        double y = (use_odom_topics || use_static_map) ? obList.obstacles[i].cirGroup.packedCirs[j].center.y : cir_obs[i]->cirGroup.packedCirs[j].center.y;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        double radius = obList.obstacles[i].cirGroup.packedCirs[j].radius;
        ROS_INFO("x: %f y: %f radius: %f", x, y, radius);
        
        //ROS_INFO("cir_obs[%i]->packedCirs[%i].center.x: %f cir_obs[%i]->packedCirs[%i].center.y: %f cir_obs[%i]->packedCirs[%i].cir.radius: %f", i, j, x, i, j, y, i, j, radius);

        // scale values are the diameter so use the radius*2
        marker.scale.x = radius*2.00f;
        marker.scale.y = radius*2.00f;
        marker.scale.z = 0.05;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1;
        marker.lifetime = ros::Duration(rvizLifetime);

        result.push_back(marker);
      }
    }
  }
  else
  {
    //ROS_WARN("Global grid dimensions wrong");
  }
  
  //ROS_INFO("Exiting convertObsToMarkers");
  return result;
}


/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  //addHardCodedStaticObs();
  /*for(int i=0;i<dynamicObsList.obstacles.size();i++)
  {
    ROS_INFO("Ob %i at (%f,%f)", i, dynamicObsList.obstacles[i].fitCir.center.x, dynamicObsList.obstacles[i].fitCir.center.y); 
  }*/
  //ROS_INFO("List size: %i", (int)dynamicObsList.obstacles.size());
  
  ROS_INFO("In publishList, staticObsList.size(): %i", (int)staticObsList.obstacles.size());
  ROS_INFO("In publishList, dynamicObsList.size(): %i", (int)dynamicObsList.obstacles.size());
  list.obstacles.clear();
  for(int i=0;i<staticObs.size();i++)
  {
    list.obstacles.push_back(staticObsList.obstacles[i]);
  }
  
  for(int i=0;i<dynamicObsList.obstacles.size();i++)
  {
    list.obstacles.push_back(dynamicObsList.obstacles[i]);
  }


  pub_obj.publish(list);
} //End sendList



void publishMarkers(const ros::TimerEvent& e)
{
  ROS_INFO("In publishMarkers");
  
  // Publish a single Marker
  visualization_msgs::MarkerArray result;

  ROS_INFO("dynamicObsList.size(): %i staticObsList.size: %i", (int)dynamicObsList.obstacles.size(), (int)staticObsList.obstacles.size());
  
  // Get circle markers for obstacles
  std::vector<visualization_msgs::Marker> markers = convertObsToMarkers(dynamicObsList,0);
  std::vector<visualization_msgs::Marker> markersStatic = convertObsToMarkers(staticObsList, markers.size());
  
  markers.reserve(markers.size() + markersStatic.size());
  markers.insert(markers.end(), markersStatic.begin(), markersStatic.end());

  ROS_INFO("markers.size(): %i markersStatic.size(): %i", (int)markers.size(), (int)markersStatic.size());

  markers = markersStatic;

  result.markers = markers;
  
  std::vector<visualization_msgs::Marker> texts;
  std::vector<visualization_msgs::Marker> arrows;

  if(use_odom_topics)
  {
  }

  /*
   * Make text to show the linear velocity value for each object
   * Make an arrow to show the direction of the linear velocity
   */
  for(int i=0;i<cir_obs.size();i++)
  {
    ////////ROS_INFO("Creating text and arrow for marker i: %i", i);
    ////////ROS_INFO("prev_velocities.size(): %i", (int)prev_velocities.size());
    ////////ROS_INFO("prev_velocities[%i].size(): %i", i, (int)prev_velocities[i].size());
    visualization_msgs::Marker text;
    visualization_msgs::Marker arrow;

    /*
     * Set members for text and arrow markers
     */
    text.header.stamp   = ros::Time::now();
    arrow.header.stamp  = ros::Time::now();

    text.id   = 10000 + i;
    arrow.id  = 200 + i;

    ////ROS_INFO("text.id: %i popSize: %i markers.size(): %i i: %i", text.id, populationSize, (int)markers.size(), i);
    ////ROS_INFO("arrow.id: %i popSize: %i markers.size(): %i i: %i", arrow.id, populationSize, (int)markers.size(), i);

    //text.header.frame_id  = "/map";
    //arrow.header.frame_id = "/map";
    //text.header.frame_id  = "/costmap";
    //arrow.header.frame_id = "/costmap";
    text.header.frame_id  = global_frame;
    arrow.header.frame_id = global_frame;

    text.ns   = "basic_shapes";
    arrow.ns  = "basic_shapes";

    
    text.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    arrow.type  = visualization_msgs::Marker::ARROW;

    text.action   = visualization_msgs::Marker::ADD;
    arrow.action  = visualization_msgs::Marker::ADD;

    //////////ROS_INFO("cir_obs.size(): %i", (int)cir_obs.size());
    //////////ROS_INFO("prev_velocities.size(): %i cir_obs[%i]->prevTheta.size(): %i", (int)prev_velocities.size(), i, (int)cir_obs[i]->prevTheta.size());
    
    // Set text value
    // Should I use cir_obs->vel?
    if(prev_velocities[prev_velocities.size()-1].size() > 0)
    {
      text.text = std::to_string(prev_velocities[prev_velocities.size()-1][i].v);
    }
    else
    {
      arrow.scale.x = 0.1;
    }

    // Set arrow points, x is the length, y is the width
    // The length has a minimum value
    arrow.scale.x = prev_velocities[prev_velocities.size()-1][i].v < min_arrow_len ? min_arrow_len : prev_velocities[prev_velocities.size()-1][i].v;
    arrow.scale.y = 0.1;
    
    // Set poses
    text.pose.position.x = cir_obs[i]->cirGroup.fitCir.center.x;
    text.pose.position.y = cir_obs[i]->cirGroup.fitCir.center.y;
    //text.pose = markers[i].pose;
    
    text.pose.position.z = 0.1;
    text.color.r = 0;
    text.color.g = 0;
    text.color.b = 1;
    text.color.a = 1;
    text.scale.z = 0.25;
    text.lifetime = ros::Duration(rvizLifetime);

    arrow.pose = text.pose;
    
    // Set the arrow orientation
    //////////ROS_INFO("cir_obs.size(): %i prevTheta.size(): %i index: %i", (int)cir_obs.size(), (int)cir_obs[i]->prevTheta.size(), (int)cir_obs[i]->prevCirs.size()-1);

    double theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : 0;
    //////////ROS_INFO("prevTheta.size(): %i theta: %f", (int)cir_obs[i]->prevTheta.size(), theta);
    tf::Quaternion q = tf::createQuaternionFromYaw(theta);
    tf::quaternionTFToMsg(q, arrow.pose.orientation);
    
   
    arrow.color.r = 1;
    arrow.color.g = 0;
    arrow.color.b = 0;
    arrow.color.a = 1;
    arrow.lifetime = ros::Duration(rvizLifetime);
    
    // Push onto texts 
    texts.push_back(text);
    arrows.push_back(arrow);
  }


  //////////ROS_INFO("texts.size(): %i", (int)texts.size());

  result.markers.insert(std::end(result.markers), std::begin(texts), std::end(texts));  
  result.markers.insert(std::end(result.markers), std::begin(arrows), std::end(arrows));  

  // Create a text marker to show the number of obstacles
  visualization_msgs::Marker text;
  text.header.stamp   = ros::Time::now();
  text.header.frame_id  = global_frame;
  text.id       = 70000;
  text.ns       = "basic_shapes";
  text.type     = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action   = visualization_msgs::Marker::ADD;
  std::stringstream num_obs;
  num_obs<<"Number of obstacles: "<<(int)cirGroups.size();
  text.text = num_obs.str();

  // Set poses
  text.pose.position.x = 0;
  text.pose.position.y = -3.5;
  text.pose.position.z = 0.1;
  text.color.r = 0;
  text.color.g = 0;
  text.color.b = 0;
  text.color.a = 1;
  text.scale.z = 0.75;
  text.lifetime = ros::Duration(rvizLifetime);

  result.markers.push_back(text);
  /*for(int i=0;i<result.markers.size();i++)
  {
    ROS_INFO("Marker %i, id: %i", i, result.markers[i].id);
  }*/
  
  // Draw the polygon lines
  /*result.markers.push_back(polygonLines);
  for(int i=0;i<pLines.size();i++)
  {
    result.markers.push_back(pLines[i]);
  }*/
  /*////////ROS_INFO("cLines.size(): %i", (int)cLines.size());
  for(int i=0;i<cLines.size();i++)
  {
    ////////ROS_INFO("cLines[%i]: (%f,%f)", i, cLines[i].pose.position.x, cLines[i].pose.position.y);
    result.markers.push_back(cLines[i]);
  }*/
  //////////ROS_INFO("publishMarkers polygonLines.size(): %i", (int)polygonLines.points.size());

  //ROS_INFO("result.markers.size(): %i", (int)result.markers.size());

  pub_rviz.publish(result);
  //ROS_INFO("Exiting publishMarkers");
}


int getClosestPrev(Circle m, std::vector<Circle> N, std::vector<int> matched)
{
  ////////////ROS_INFO("In getClosestPrev");
  ////////////ROS_INFO("N.size(): %i", (int)N.size());
  int min_index = 0;
  double dist_threshold = 0.2;

  // Don't start at index 0 if it's already been matched
  while(matched[min_index])
  {
    min_index++;
  }

  std::vector<double> center;
  center.push_back(m.center.x);
  center.push_back(m.center.y);

  ////////////ROS_INFO("center: [%f, %f]", center[0], center[1]);

  // If there are circles left to match
  if(N.size() > 0)
  {

    // Get center and dist used as the initial minimum
    std::vector<double> prev_center;
    prev_center.push_back(N.at(min_index).center.x);
    prev_center.push_back(N.at(min_index).center.y);

    double dist = util.positionDistance(center, prev_center);
    
    ////////////ROS_INFO("Prev center: [%f, %f]", prev_center[0], prev_center[1]);
    ////////////ROS_INFO("dist: %f", dist);

    // Go through remaining potential matches, find min dist
    for(int i=min_index;i<N.size();i++)
    {
      prev_center.at(0) =  N[i].center.x;
      prev_center.at(1) =  N[i].center.y;
      ////////////ROS_INFO("Prev center: [%f, %f] dist: %f", prev_center[0], prev_center[1], util.positionDistance(center, prev_center));

      // Compare distance with  min distance, and that the target has not already been matched
      if( util.positionDistance(center, prev_center) < dist && !matched[i])
      {
        dist =  util.positionDistance(center, prev_center);
        min_index = i;
      }
    } // end for each potential match
  } // end if N.size() > 0

  else
  {
    return -1;
  }

  return min_index;
}

// ***************************************************
// Add in check for angles where tan is undefined
// ***************************************************
double getDistToFOV(Point p, double angle)
{
  /*////////ROS_INFO("In getDistToFOV");
  ////////ROS_INFO("p: (%f,%f)", p.x, p.y);
  ////////ROS_INFO("Robot pos: (%f,%f)", robotState.positions[0], robotState.positions[1]);*/
  double m = tan(angle);
  double b = robotState.positions[1] - (m*robotState.positions[0]);

  double d = fabs( (m*p.x) + p.y - b );

  //////////ROS_INFO("angle: %f m: %f b: %f d: %f", angle, m, b, d);

  d /= sqrt( pow(m,2) + pow(b,2) );

  //////////ROS_INFO("denom: %f d: %f", sqrt( pow(m,2) + pow(b,2) ), d);

  //////////ROS_INFO("Exiting getDistToFOV");
  return d;
}




std::vector<Velocity> predictVelocities(const std::vector<CircleMatch> cm, const ros::Duration d_elapsed)
{
  //ROS_INFO("In predictVelocities, d_elapsed: %f", d_elapsed.toSec());
  //ROS_INFO("cir_obs.size(): %i cm.size(): %i", (int)cir_obs.size(), (int)cm.size());

  std::vector<Velocity> result;
  
  // For each circle obstacle,
  for(int i=0;i<cir_obs.size();i++)
  {
    //ROS_INFO("CircleOb %i prevCirs.size(): %i", i, (int)cir_obs[i]->prevCirs.size());
    //////ROS_INFO("Current: (%f, %f)", cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);

    Velocity temp;

    // If prevCir is set 
    if(cir_obs[i]->prevCirs.size() > 0)
    {
      // Prev needs to be based on the global x,y
      int i_prev = cir_obs[i]->prevCirs.size()-1;
      //ROS_INFO("i_prev: %i", i_prev);
      //////ROS_INFO("Prev: (%f, %f)", cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y);
      double x_dist = cir_obs[i]->cirGroup.fitCir.center.x - cir_obs[i]->prevCirs[i_prev].center.x;
      double y_dist = cir_obs[i]->cirGroup.fitCir.center.y - cir_obs[i]->prevCirs[i_prev].center.y;
      //double dist = sqrt( pow(x_dist,2) + pow(y_dist,2) );

      double dist = util.positionDistance(cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y, cir_obs[i]->cirGroup.fitCir.center.x, cir_obs[i]->cirGroup.fitCir.center.y);

      //////ROS_INFO("cir_obs.size(): %i i: %i", (int)cir_obs.size(), i);
      std::vector<double> obPos;
      obPos.push_back(cir_obs[i]->cirGroup.fitCir.center.x);
      obPos.push_back(cir_obs[i]->cirGroup.fitCir.center.y);
      

      double theta = atan2(y_dist, x_dist);
      double dirToOb = util.findAngleFromAToB(robotState.positions, obPos);
      
      // Add on change in radius
      if(cm.size()>0)
      {
        // Get cm for ob
        for(int j=0;j<cm.size();j++)
        {
          if(cm[j].i_cir == i)
          {
            //////////ROS_INFO("dist: %f delta_r: %f", dist, cm[j].delta_r);
            // If the obstacle is moving towards the robot, then add delta_r
            if(viewMinMax.size() > 1)
            {
              double dMin = getDistToFOV(cir_obs[i]->cirGroup.fitCir.center, viewMinMax[0]);
              double dMax = getDistToFOV(cir_obs[i]->cirGroup.fitCir.center, viewMinMax[1]);
              //////////ROS_INFO("viewMinMax[0]: %f viewMinMax[1]: %f dMin: %f dMax: %f r: %f", viewMinMax[0], viewMinMax[1], dMin, dMax, cir_obs[i]->cirGroup.fitCir.radius);
              if(dMin < cir_obs[i]->cirGroup.fitCir.radius+0.5 || dMax < cir_obs[i]->cirGroup.fitCir.radius+0.5)
              {
                dist -= cm[j].delta_r;
              }
              else
              {
                dist += cm[j].delta_r;
              }
            }
          }
        }
      }
      //////ROS_INFO("After if(cm.size()>0)");

      /*
       * Set angular velocity to 0 because we currently don't use it at all
       * In the future, i_prev won't correspond to the previous theta value 
       * so this will need to be reworked
       */
      //////////ROS_INFO("cir_obs[%i]->prevTheta.size(): %i i_prev: %i", i, (int)cir_obs[i]->prevTheta.size(), i_prev);
      //double w = util.displaceAngle(theta, cir_obs[i]->prevTheta[i_prev]) / d_elapsed.toSec();
      double w = 0;

      double linear_v = (dist / d_elapsed.toSec());

      // Xdot and Ydot should be based on overall linear speed
      temp.vx = linear_v*cos(theta);
      temp.vy = linear_v*sin(theta);
      temp.v  = linear_v;
      temp.w  = w;
      //////////ROS_INFO("dist: %f t: %f vx: %f vy: %f speed: %f theta: %f w: %f", dist, d_elapsed.toSec(), temp.vx, temp.vy, linear_v, theta, w);

      if(dist > 0.075)
      {
        //////////ROS_INFO("Setting moving = true, dist: %f", dist);
        cir_obs[i]->moving = true;
      }
      
      predicted_velocities.push_back(temp);
    }
    else
    {
      //////////ROS_INFO("No previous circles");
      temp.vx = 0;
      temp.vy = 0;
      temp.v  = 0;
      temp.w  = 0;
    }

    // Push the circle's velocity onto the result
    result.push_back(temp);
  } // end for

  //////ROS_INFO("Exiting predictVelocities");
  return result;
}


std::vector<double> predictTheta()
{
  //////////ROS_INFO("In predictTheta");
  std::vector<double> result;

  // For each circle obstacle
  for(int i=0;i<cir_obs.size();i++)
  {
    // This should be initialized better, no reason to assume 
    // an obstacle starts at orientation 0
    double theta=initial_theta;

    // If prevCir is set
    if(cir_obs[i]->prevCirs.size() > num_costmap_freq_theta)
    {
      int i_prev = cir_obs[i]->prevCirs.size()-num_costmap_freq_theta;
      //int i_prev = cir_obs[i]->i_prevThetaCir;
      /*//////////ROS_INFO("i_prev: %i", i_prev);
      //////////ROS_INFO("Prev: (%f, %f) Current: (%f, %f)", cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y, cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);*/
      double x_dist = cir_obs[i]->cirGroup.fitCir.center.x - cir_obs[i]->prevCirs[i_prev].center.x;
      double y_dist = cir_obs[i]->cirGroup.fitCir.center.y - cir_obs[i]->prevCirs[i_prev].center.y;
      //////////ROS_INFO("x_dist: %f y_dist: %f", x_dist, y_dist);

      theta = atan2(y_dist, x_dist);
    }
    
    result.push_back(theta); 
  } // end for each circle obstacle

  //////////ROS_INFO("Exiting predictTheta");
  return result;
}


CircleOb* createCircleOb(CircleGroup temp)
{
  ////ROS_INFO("In createCircleOb, STATE_SIZE: %i", STATE_SIZE);

  CircleOb* result = new CircleOb;
  result->cirGroup = temp;

  BFL::ColumnVector prior_mu(STATE_SIZE);
  prior_mu(1) = temp.fitCir.center.x;
  prior_mu(2) = temp.fitCir.center.y;
  prior_mu(3) = 0;
  prior_mu(4) = 0;
  MatrixWrapper::SymmetricMatrix prior_cov(STATE_SIZE);
  prior_cov = 0.0;
  prior_cov(1,1) = PRIOR_COV_X;
  prior_cov(2,2) = PRIOR_COV_Y;
  prior_cov(3,3) = PRIOR_COV_VX;
  prior_cov(4,4) = PRIOR_COV_VY;
  BFL::Gaussian* prior = new BFL::Gaussian(prior_mu, prior_cov);

  CircleFilter* cir_filter = new CircleFilter(STATE_SIZE, prior, sys_pdf, meas_pdf);
  result->kf = cir_filter;

  ////ROS_INFO("Exiting createCircleOb");
  return result;
}





void transformCostmap(nav_msgs::OccupancyGrid& g)
{
  ////ROS_INFO("In transformCostmap");
  tf::Vector3 p(g.info.origin.position.x, g.info.origin.position.y, 0);
  float res = g.info.resolution;
  int w = g.info.width;
  int h = g.info.height;
  ////ROS_INFO("p: (%f,%f) w: %i h: %i c_max (w): %f r_max (h): %f", p.getX(), p.getY(), w, h , global_costmap.info.resolution / w, global_costmap.info.resolution / h);

  tf::Vector3 p_global(global_costmap.info.origin.position.x, global_costmap.info.origin.position.y, 0);
  ////ROS_INFO("p_global: (%f,%f)", p_global.getX(), p_global.getY());

 
  float delta_x = g.info.origin.position.x - global_costmap.info.origin.position.x;
  float delta_y = g.info.origin.position.y - global_costmap.info.origin.position.y;
  int i_dx = delta_x / res;
  int i_dy = (delta_y / res) * global_costmap.info.width;
  ////ROS_INFO("delta_x: %f delta_y: %f i_dx: %i i_dy: %i", delta_x, delta_y, i_dx, i_dy);

  float x_global_max = (global_costmap.info.width * global_costmap.info.resolution) + global_costmap.info.origin.position.x;
  float y_global_max = (global_costmap.info.height * global_costmap.info.resolution) + global_costmap.info.origin.position.y;
  float x_global_min = global_costmap.info.origin.position.x;
  float y_global_min = global_costmap.info.origin.position.y;

  ////ROS_INFO("x_global_max: %f y_global_max: %f", x_global_max, y_global_max);

  // For each point on the new grid g, 
  // Set the value on the corresponding cell in the global grid
  for(int i=0;i<g.data.size();i++)
  {
    // Get row,column values for index i
    float r = (i / w) * res;
    float c = ((i % w)+1) * res;
    ////ROS_INFO("r: %f c: %f", r, c);

    // Convert to coordinates
    float x = c;
    float y = r;
    // Convert to coordinates relative to grid origin
    x += p.getX();
    y += p.getY();

   

    // Get index on global costmap
    int c_global = ((i % g.info.width) % global_costmap.info.width) + 1;
    // divide to get rid of remainder, then re-multiply by width
    int r_global = (i / g.info.width) * global_costmap.info.width;
    
    ////ROS_INFO("Before considering origin, c_global: %i r_global: %i", c_global, r_global);

    c_global += i_dx ;//<= -(c/res) ? -(c/res) * global_costmap.info.width : i_dx;
    r_global += i_dy;

    //int i_global = (c_global > -1 && c_global < 901 && r_global > -1 && r_global < 10001) ? r_global + c_global : -1;
    int i_global = r_global + c_global;
   
    // Print info 
    ////ROS_INFO("x_global_min, max: (%f,%f) y_global_min, max: (%f,%f)", x_global_min, x_global_max, y_global_min, y_global_max);
    ////ROS_INFO("x: %f y: %f c_global: %i r_global: %i i: %i i_global: %i global.size(): %i", x, y, c_global, r_global, i, i_global, (int)global_costmap.data.size());

    if(x >= x_global_min && x < x_global_max && y >= y_global_min && y < y_global_max)
    {
      ////ROS_INFO("Good");
      i_global = r_global + c_global;
    }
    else
    {
      ////ROS_INFO("Bad");
      i_global = -1;
    }


    if(i_global > -1 && i_global < global_costmap.data.size())
    {
      global_costmap.data[i_global] = g.data[i];
    }
  }
  //////////ROS_INFO("global_costmap.data.size(): %i", (int)global_costmap.data.size());
}


void accumulateCostmaps(const nav_msgs::OccupancyGrid& g1, const nav_msgs::OccupancyGrid& g2, nav_msgs::OccupancyGrid& result)
{
  ////ROS_INFO("In asscumulateCostmaps(OccupancyGrid, OccupancyGrid, OccupancyGrid)");
  ////ROS_INFO("g1.data.size(): %i g2.data.size(): %i", (int)g1.data.size(), (int)g2.data.size());
  ////ROS_INFO("g1.w: %i g1.h: %i g2.w: %i g2.h: %i", g1.info.width, g1.info.height, g2.info.width, g2.info.height);
  result = g1;

  /*float ox = g2.info.origin.position.x - g1.info.origin.position.x;
  float oy = g2.info.origin.position.y - g1.info.origin.position.y;

  int i_x_offset = ox / g2.info.resolution;
  int x_off_g1 = ox > 0 ? i_x_offset : 0;
  int x_off_g2 = ox < 0 ? i_x_offset : 0;

  int i_y_offset = (oy / g2.info.resolution) * g1.info.width;
  int y_off_g1 = ox > 0 ? i_y_offset : 0;
  int y_off_g2 = ox < 0 ? i_y_offset : 0;

  int i_offset_g  = x_off_g1 + y_off_g1;
  int i_offset_gg = x_off_g2 + y_off_g2;

  ////////ROS_INFO("ox: %f oy: %f i_x_offset: %i x_off_g1: %i x_off_g2: %i i_y_offset: %i y_off_g1: %i y_off_g2: %i i_offset_g: %i i_offset_gg: %i", 
      ox, oy, i_x_offset, x_off_g1, x_off_g2, i_y_offset, y_off_g1, y_off_g2, i_offset_g, i_offset_gg);*/

  //////////ROS_INFO("g1.info.width: %i g1.info.height: %i", g1.info.width, g1.info.height);
  //////////ROS_INFO("Before for loops, result.size(): %i", (int)result.data.size());
  for(int c=0;c<g1.info.height && c<g2.info.height;c++)
  {
    int c_offset = g1.info.width < g2.info.width ? g1.info.width*c : g2.info.width;
    for(int r=0;r<g1.info.width && r<g2.info.width;r++)
    {
      
      //result.data[c_offset + r] = (g1.data[c_offset + r + i_offset_g] | g2.data[c_offset + r + i_offset_gg]);
      result.data[c_offset + r] = (g1.data[c_offset + r] | g2.data[c_offset + r]);
      /*if(!g1.data[r_offset + c] && !g2.data[r_offset + c] && result.data[r_offset+c])
      {
        ////////ROS_INFO("g1.data[%i]: %i g2.data: %i result: %i", r_offset+c, g1.data[r_offset+c], g2.data[r_offset+c], result.data[r_offset+c]);
      }*/
    }
  }

  //////////ROS_INFO("After for loops, result.size(): %i", (int)result.data.size());
}

void accumulateCostmaps(const nav_msgs::OccupancyGrid gi, const std::vector<nav_msgs::OccupancyGrid> prev_grids, nav_msgs::OccupancyGrid& result)
{
  //////////ROS_INFO("In consolidateCostmaps(OccupancyGrid, vector<OccupancyGrid>, OccupancyGrid)");
  //////////ROS_INFO("gi.size(): %i", (int)gi.data.size());
  
  if(prev_grids.size() == 0)
  {
    result = gi;
  }
  else
  {
    nav_msgs::OccupancyGrid temp = gi;

    for(int i=0;i<prev_grids.size();i++)
    {
      ////////////ROS_INFO("Consolidating with previous grid %i, prev_grid[%i].size(): %i", i, i, (int)prev_grids[i].data.size());
      accumulateCostmaps(temp, prev_grids[i], result);
      ////////////ROS_INFO("New result size: %i", (int)result.data.size());
      temp = result;
    }
  }

  //////////ROS_INFO("result.size(): %i", (int)result.data.size());
}


/*
 * Returns a vector of CircleMatches which relate a circle in the latest grid to a circle in the previous grid
 */
std::vector<CircleMatch> matchCircles(std::vector<CircleGroup> cirs, std::vector<CircleGroup> targets)
{
  //////////ROS_INFO("In matchCircles");
  //////////ROS_INFO("cirs.size(): %i targets.size(): %i", (int)cirs.size(), (int)targets.size());
  for(int i=0;i<cirs.size();i++)
  {
    //////////ROS_INFO("Circle %i: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
  }
  for(int i=0;i<targets.size();i++)
  {
    //////////ROS_INFO("Target %i: (%f,%f)", i, targets[i].center.x, targets[i].center.y);
  }
  std::vector<CircleMatch> result;
  std::vector<int> matched_targets(targets.size());
  std::vector<CircleMatch> initial;

  // 2D array for dists of each cir to each target
  // [target][cir i dist]
  std::vector< std::vector<double> > dists;

  std::vector<CircleMatch> all_dists;

  // Initial matching
  for(int i=0;i<targets.size();i++)
  {
    std::vector<double> target_dists;
    for(int j=0;j<cirs.size();j++)
    {
      target_dists.push_back(util.positionDistance(cirs[j].fitCir.center.x, cirs[j].fitCir.center.y, targets[i].fitCir.center.x, targets[i].fitCir.center.y));

      // Create CircleMatch object
      CircleMatch temp;
      temp.i_cir = j;
      temp.i_prevCir = i;
      temp.dist = util.positionDistance(cirs[j].fitCir.center.x, cirs[j].fitCir.center.y, targets[i].fitCir.center.x, targets[i].fitCir.center.y);
      temp.delta_r = fabs(cirs[j].fitCir.radius - targets[i].fitCir.radius);

      all_dists.push_back(temp);
    }

    dists.push_back(target_dists);
  } // end getting dist values

  //////////ROS_INFO("all_dists.size(): %i", (int)all_dists.size());

  // Sort dist values
  std::sort(all_dists.begin(), all_dists.end(), util.compareCircleMatches);

  // Print all potential matches
  for(int i=0;i<all_dists.size();i++)
  {
    //////////ROS_INFO("all_dists %i: i_cirs: %i targets: %i dist: %f delta_r: %f", i, all_dists[i].i_cirs, all_dists[i].i_prevCir, all_dists[i].dist, all_dists[i].delta_r);
  }
    
  int i=0;
  while(i < all_dists.size())
  {
    //////////ROS_INFO("all_dists %i: i_cirs: %i targets: %i dist: %f delta_r: %f", i, all_dists[i].i_cirs, all_dists[i].i_prevCir, all_dists[i].dist, all_dists[i].delta_r);

    // Check if this is a legitimate match based on dist and radius change
    if(all_dists[i].dist < dist_threshold && all_dists[i].delta_r < radius_threshold)
    {
      //////////ROS_INFO("Legitimate match");

      // Now check that the target or circle hasn't been matched already
      bool prev_matched = false;
      for(int r=0;r<result.size();r++)
      {
        //////////ROS_INFO("r: %i result[%i].i_cirs: %i result[%i]: %i", r, r, result[r].i_cirs, r, result[r].i_cirs);
        if(result[r].i_cir == all_dists[i].i_cir || result[r].i_prevCir == all_dists[i].i_prevCir)
        {
          //////////ROS_INFO("Previously matched!");
          prev_matched = true;
          break;
        }
      } // end for

      if(!prev_matched)
      {
        //////////ROS_INFO("Not previously matched!");
        result.push_back(all_dists[i]);
        all_dists.erase(all_dists.begin()+i);
        i--;
      }
    } // end if legitimate match
    else
    {
      //////////ROS_INFO("Match not legitimate");
    }

    i++;
  } // end for all_dists values


  //////////ROS_INFO("Exiting matchCircles");
  return result;
}


void deleteOldObs(std::vector<CircleMatch> cm)
{
  // Detect whether any targets could not be matched
  std::vector<int> matched_tar(prev_valid_cirs.size(), 0);
  for(int j=0;j<cm.size();j++)
  {
    matched_tar[ cm[j].i_prevCir ]++;
  }

  
  // Go through and delete obstacle filters
  int index_cir_obs=0;
  for(int i=0;i<prev_valid_cirs.size();i++)
  {
    if(i < matched_tar.size() && !matched_tar[i])
    {
      //////////ROS_INFO("matched_tar[%i]: %i", i, matched_tar[i]);
      //////////ROS_INFO("Deleting filter at index %i!", index_cir_obs);
      delete cir_obs[index_cir_obs];
      cir_obs.erase(cir_obs.begin()+index_cir_obs);

      // Don't decrement i because we are looping through matched_tar, not cir_obs
      index_cir_obs--;
    }
    index_cir_obs++;
  }
}

void addNewObs(std::vector<CircleMatch> cm, std::vector<CircleGroup> cirs)
{
  // Determine if we need to modify cir_obs
  // If any new circles could not be matched
  std::vector<int> matched_cirs(cirs.size(), 0);
  for(int i=0;i<cm.size();i++)
  {
    matched_cirs[ cm[i].i_cir ]++;
  }

  // Create new filters
  for(int i=0;i<cirs.size();i++)
  {
    //////////ROS_INFO("matched_cirs[%i]: %i", i, matched_cirs[i]);
    if(!matched_cirs[i])
    {
      //////////ROS_INFO("Creating new filter! Center: (%f,%f) Radius: %f", cirs[i].center.x, cirs[i].center.y, cirs[i].radius);
      CircleOb* temp = createCircleOb(cirs[i]);
      //////////ROS_INFO("i: %i cir_obs.size(): %i", i, (int)cir_obs.size());
      //cir_obs.insert(cir_obs.begin()+i, temp);
      cir_obs.push_back(temp);
    }
  }
}


/*
 * Compare new circles to old circles and determine matches
 * cir_obs[i]->cir values are set here
 */
std::vector<CircleMatch> dataAssociation(std::vector<CircleGroup> cirs)
{
  //////////ROS_INFO("Starting data association");
  //////////ROS_INFO("cirs.size(): %i prev_valid_cirs: %i cir_obs: %i", (int)cirs.size(), (int)prev_valid_cirs.size(), (int)cir_obs.size());
  
  std::vector<CircleMatch> cm = matchCircles(cirs, prev_valid_cirs);
  //////////ROS_INFO("cm.size(): %i", (int)cm.size());
  //////////ROS_INFO("Matching result:");
 
  std::vector<CircleGroup> copy = cirs;
  for(int i=0;i<cm.size();i++)
  {
    //////////ROS_INFO("Match %i: i_cirs: %i i_prevCir: %i dist: %f delta_r: %f", i, cm[i].i_cirs, cm[i].i_prevCir, cm[i].dist, cm[i].delta_r);
    
    /*
     * This may need to be looked at later on
     * what if cir_obs.size() == 0? or less than cm.size()? or cm[i].i_prevCir?
     * cir_obs doesn't get new objects pushed on until addNewObs is called
     */
    // Set cir value for this circle obstacle!
    if(cir_obs.size() > cm[i].i_prevCir)
    {
      cir_obs[cm[i].i_prevCir]->cirGroup = copy[cm[i].i_cir];
      //////////ROS_INFO("Setting cir_obs[%i]->cir to (%f,%f)", cm[i].i_prevCir, copy[cm[i].i_cirs].center.x, copy[cm[i].i_cirs].center.y);
    }
  } // end for each potential match
 
  // Delete and add filters based on matching results
  deleteOldObs(cm); 
  addNewObs(cm, cirs);


  /*//////////ROS_INFO("Done with data association, cir_obs.size(): %i", (int)cir_obs.size());
  for(int i=0;i<cir_obs.size();i++)
  {
    //////////ROS_INFO("cir_obs[%i] circle: (%f,%f)", i, cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);
    if(cir_obs[i]->prevCirs.size()>0)
    {
      //////////ROS_INFO("Prev: (%f,%f)", cir_obs[i]->prevCirs[cir_obs[i]->prevCirs.size()-1].center.x, cir_obs[i]->prevCirs[cir_obs[i]->prevCirs.size()-1].center.y);
    }
    else
    {
      //////////ROS_INFO("No prev");
    }
  }*/

  return cm;
}


std::vector<CircleGroup> updateKalmanFilters(std::vector<CircleGroup> cirs, std::vector<CircleMatch> cm)
{
  ////ROS_INFO("In updateKalmanFilters");

  std::vector<CircleGroup> result;
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("Updating circle filter %i", i);

    // Prediction
    if(prev_velocities.size() > 0 && prev_velocities[prev_velocities.size()-1].size() > i)
    {
      u[0] = prev_velocities[prev_velocities.size()-1][i].vx;
      u[1] = prev_velocities[prev_velocities.size()-1][i].vy;
      u[2] = 0;
      u[3] = 0;
    }
    else
    {
      u[0] = 0;
      u[1] = 0;
      u[2] = 0;
      u[3] = 0;
    }
    
    // Measurement 
    MatrixWrapper::ColumnVector y(STATE_SIZE);
    y[0] = cir_obs[i]->cirGroup.fitCir.center.x;
    y[1] = cir_obs[i]->cirGroup.fitCir.center.y;
    for(int i=2;i<STATE_SIZE;i++)
    {
      y[i] = 0;
    }
    ////ROS_INFO("Measurement: (%f, %f, %f, %f)", y[0], y[1], y[2], y[3]);
    ////ROS_INFO("Prediction: (%f, %f, %f, %f)", u[0], u[1], u[2], u[3]);


    // Update the Kalman filter
    cir_obs[i]->kf->update(u, y);
    ////ROS_INFO("Done calling update");
    cir_obs[i]->kf->printPosterior();
    ////ROS_INFO("Done calling printPosterior");
    
    // Set new circle center
    MatrixWrapper::ColumnVector mean = cir_obs[i]->kf->posterior->ExpectedValueGet();
    cir_obs[i]->cirGroup.fitCir.center.x = mean[0];
    cir_obs[i]->cirGroup.fitCir.center.y = mean[1];

    // Push on resulting circle
    result.push_back(cir_obs[i]->cirGroup);

    // Push back center data for logging
    cirs_pos.push_back(cir_obs[i]->cirGroup.fitCir);
  }
  
  /*////////ROS_INFO("After kalman filter:");
  for(int i=0;i<cir_obs.size();i++)
  {
    ////////ROS_INFO("cir_obs[%i]->cirGroup.fitCir.center.x: %f cir_obs[%i]->cirGroup.fitCir.center.y: %f cir_obs[%i]->cir.radius: %f", i, cir_obs[i]->cirGroup.fitCir.center.x, i, cir_obs[i]->cirGroup.fitCir.center.y, i, cir_obs[i]->cir.radius);
  }*/


  ////ROS_INFO("Exiting updateKalmanFilters");
  return result;
}



Point getGlobalCoords(const Circle& cir)
{
  Point result;

  double x_origin = global_grid.info.origin.position.x / costmap_res;
  double y_origin = global_grid.info.origin.position.y / costmap_res;

  result.x = (cir.center.x + x_origin) * costmap_res;
  result.y = (cir.center.y + y_origin) * costmap_res;
  

  return result;
}


/*
 * This crops out the costmap values behind the robot.
 * This is useful for robots that cannot see behind them, such as the turtlebot
 * or any other robot using an rgb-d camera
 */ 
void cropCostmap(const nav_msgs::OccupancyGridConstPtr grid, nav_msgs::OccupancyGrid& result)
{
  //////////ROS_INFO("In cropCostmap");

  // This are the static bounds
  // a is the lower-left corner, then go in cw order
  float res = grid->info.resolution;
  float w = grid->info.width  * res;
  float h = grid->info.height * res;
  tf::Transform tf_g_to_base = tf_base_to_global.inverse();

  float x_min=ranges[0].min;
  float y_min=ranges[1].min;
  float x_max=ranges[0].max;
  float y_max=ranges[1].max;
  
  //////////ROS_INFO("costmap origin: (%f,%f) width: %i height: %i resolution: %f w: %f h: %f", grid->info.origin.position.x, grid->info.origin.position.y, grid->info.width, grid->info.height, grid->info.resolution, w, h);

  // a = costmap origin
  tf::Vector3 p_a(grid->info.origin.position.x, grid->info.origin.position.y, 0);

  // b = top-left
  tf::Vector3 p_b(p_a.getX(), p_a.getY()+h, 0);

  // c = top-right
  tf::Vector3 p_c(p_a.getX()+w, p_a.getY()+h, 0);

  // d = bottom-right
  tf::Vector3 p_d(p_a.getX()+w, p_a.getY(), 0);


  std::vector<tf::Vector3> p_vec, p_w_vec;
  p_vec.push_back(p_a);
  p_vec.push_back(p_b);
  p_vec.push_back(p_c);
  p_vec.push_back(p_d);

  for(int i=0;i<p_vec.size();i++)
  {
    //////////ROS_INFO("p_vec[%i]: (%f,%f)", i, p_vec[i].getX(), p_vec[i].getY());
    tf::Vector3 p_i_w = tf_base_to_global * p_vec[i];
    p_w_vec.push_back(p_i_w);
    //////////ROS_INFO("p_w_vec[%i]: (%f,%f)", i, p_w_vec[i].getX(), p_w_vec[i].getY());
  }


  // Check if we need to crop
  float delta_x_min = fabs(x_min - p_a.getX());
  float delta_x_max = fabs(x_max - p_c.getX());
  float delta_y_min = fabs(y_min - p_a.getY());
  float delta_y_max = fabs(y_max - p_c.getY());
  //////////ROS_INFO("delta_x_min: %f delta_x_max: %f delta_y_min: %f delta_y_max: %f", delta_x_min, delta_x_max, delta_y_min, delta_y_max);
  
  int x_min_ind = p_a.getX() < x_min ? delta_x_min / res : 0;
  int x_max_ind = p_c.getX() > x_max ? delta_x_max / res : 0;
  int y_min_ind = p_a.getY() < y_min ? delta_y_min / res : 0;
  int y_max_ind = p_c.getY() > y_max ? delta_y_max / res : 0;
  //////////ROS_INFO("x_min_ind: %i x_max_ind: %i y_min_ind: %i y_max_ind: %i", x_min_ind, x_max_ind, y_min_ind, y_max_ind);

  int width_new   = grid->info.width  - x_max_ind - x_min_ind;
  int height_new  = grid->info.height - y_max_ind - y_min_ind;
  /*////////ROS_INFO("width_new: %i height_new: %i", width_new, height_new);
  ////////ROS_INFO("grid->info.height-y_max_ind: %i", grid->info.height-y_max_ind);
  ////////ROS_INFO("grid->info.width-x_max_ind: %i", grid->info.width-x_max_ind);*/
  for(int c=y_min_ind;c<grid->info.height-y_max_ind;c++)
  {
    int c_offset = (c*grid->info.width);
    for(int r=x_min_ind;r<grid->info.width-x_max_ind;r++)
    {
      //////////ROS_INFO("c: %i c_offset: %i r: %i total: %i", c, c_offset, r, c_offset+r);
      result.data.push_back(grid->data[c_offset + r]);
    }
  }

  result.info = grid->info;
  result.info.width = width_new;
  result.info.height = height_new;
  result.info.origin.position.x += (x_min_ind*res);
  result.info.origin.position.y += (y_min_ind*res);

  //////////ROS_INFO("result.info.width: %i result.info.height: %i", result.info.width, result.info.height);
  //////////ROS_INFO("result.info.origin.position: (%f,%f)", result.info.origin.position.x, result.info.origin.position.y);
}


void removeWallObs(std::vector<Circle>& cirs)
{
  int i=0;
  if(ranges.size() > 1)
  {
    while(i<cirs.size())
    {
      Point c = getGlobalCoords(cirs[i]);
      //////////ROS_INFO("Circle %i global coordinates: (%f, %f)", i, c.x, c.y);
      
      if( c.x < ranges[0].min || c.x > ranges[0].max ||
          c.y < ranges[1].min || c.y > ranges[1].max )
      {
        //////////ROS_INFO("Obstacle is too close to boundary, discarding it");
        cirs.erase(cirs.begin()+i);
        i--;
      }
      else
      {
        //////////ROS_INFO("Keeping obstacle");
      }
      i++;
    }
  }
  else
  {
    //////////ROS_WARN("Not removing wall obstacles, ranges.size(): %i", (int)ranges.size());
  }
}


void halfCostmap(const nav_msgs::OccupancyGridConstPtr grid, nav_msgs::OccupancyGrid& result)
{
  //////////ROS_INFO("In halfCostmap");
  //////////ROS_INFO("Costmap info: width: %i height: %i origin: (%f,%f)", grid->info.width, grid->info.height, grid->info.origin.position.x, grid->info.origin.position.y);
  
  for(int c=grid->info.height/2;c<grid->info.height;c++)
  {
    int c_offset = (c*grid->info.width);// + grid->info.width/2;
    for(int r=grid->info.width/2;r<grid->info.width;r++)
    {
      //////////ROS_INFO("c: %i c_offset: %i r: %i total: %i", c, c_offset, r, c_offset+r);
      result.data.push_back(grid->data[c_offset + r]);
    }
  }

  result.info = grid->info;
  result.info.width /= 2;
  result.info.height /= 2;
  result.info.origin.position.x += (result.info.width * result.info.resolution);
  result.info.origin.position.y += (result.info.height * result.info.resolution);
  
  //////////ROS_INFO("Half Costmap info: width: %i height: %i origin: (%f,%f)", result.info.width, result.info.height, result.info.origin.position.x, result.info.origin.position.y);

  //////////ROS_INFO("Exiting halfCostmap");
}


void initGlobalMap()
{
  //////////ROS_INFO("In initGlobalMap");
  if(ranges.size() > 1)
  {
    //ROS_INFO("costmap_res: %f", costmap_res);
    global_costmap.info.resolution = costmap_res > 0 ? costmap_res : 0.05;

    global_costmap.info.origin.position.x = 0;
    global_costmap.info.origin.position.y = 0;

    global_costmap.info.width             = ((ranges[0].max - ranges[0].min) / global_costmap.info.resolution)+1;
    global_costmap.info.height            = ((ranges[1].max - ranges[1].min) / global_costmap.info.resolution)+1;


    size_t size = global_costmap.info.width * global_costmap.info.height;
    global_costmap.data.reserve(size);
    for(int i=0;i<size;i++)
    {
      global_costmap.data.push_back(-1);
    }
    
    //////////ROS_INFO("Global CM origin: (%f,%f) w: %i h: %i size: %i", global_costmap.info.origin.position.x, global_costmap.info.origin.position.y, global_costmap.info.width, global_costmap.info.height, (int)global_costmap.data.size());
  }
  else
  {
    ////////ROS_ERROR("Cannot set global costmap until rosparams are loaded");
  }
}


void setRobotPos(const ramp_msgs::MotionState& ms)
{
  ////ROS_INFO("In setRobotPos");
  ////ROS_INFO("ms: (%f,%f,%f)", ms.positions[0], ms.positions[1], ms.positions[2]);
  robotState.positions.clear();
  
  // Set new position
  robotState.positions.push_back(ms.positions[0]);
  robotState.positions.push_back(ms.positions[1]);
  robotState.positions.push_back(ms.positions[2]);

  viewMinMax.clear();

  viewMinMax.push_back( util.displaceAngle(robotState.positions[2], -fovAngle/2.0) );
  viewMinMax.push_back( util.displaceAngle(robotState.positions[2], fovAngle/2.0) );
  //////////ROS_INFO("viewMinMax: [%f,%f]", viewMinMax[0], viewMinMax[1]);

  /* 
   * Create markers in rviz to display this information
   */
  visualization_msgs::MarkerArray viewingInfo;

  // Robot position
  visualization_msgs::Marker robotPos;
  robotPos.type = visualization_msgs::Marker::SPHERE;
  robotPos.action = visualization_msgs::Marker::ADD;
  robotPos.pose.position.x = robotState.positions[0];
  robotPos.pose.position.y = robotState.positions[1];
  
  robotPos.scale.x = 0.4;
  robotPos.scale.y = 0.4;
  robotPos.scale.z = 0.1;
  
  robotPos.color.r = 0;
  robotPos.color.g = 0;
  robotPos.color.b = 0;
  robotPos.color.a = 1;

  robotPos.lifetime = ros::Duration(20);
  viewingInfo.markers.push_back(robotPos);

  // Robot orientation
  visualization_msgs::Marker robotTheta;
  robotTheta.type = visualization_msgs::Marker::ARROW;
  robotTheta.action = visualization_msgs::Marker::ADD;
  robotTheta.pose = robotPos.pose;
  
  // Set scale
  robotTheta.scale.x = 0.5;
  robotTheta.scale.y = 0.1;

  // Set orientation
  tf::Quaternion q = tf::createQuaternionFromYaw(robotState.positions[2]);
  tf::quaternionTFToMsg(q, robotTheta.pose.orientation);
  
  robotTheta.color.r = 0;
  robotTheta.color.g = 0;
  robotTheta.color.b = 0;
  robotTheta.color.a = 1;

  robotTheta.lifetime = ros::Duration(20);
  viewingInfo.markers.push_back(robotTheta);


  // Viewing min and max lines (use arrows so we only have to compute direction and not endpoints)
  visualization_msgs::Marker angleMin, angleMax;
  
  angleMin.type = visualization_msgs::Marker::ARROW;
  angleMax.type = visualization_msgs::Marker::ARROW;
  angleMin.action = visualization_msgs::Marker::ADD;
  angleMax.action = visualization_msgs::Marker::ADD;
  angleMin.pose = robotPos.pose;
  angleMax.pose = robotPos.pose;
  
  
  // Set scale
  angleMin.scale.x = 3.0;
  angleMin.scale.y = 0.05;
  angleMax.scale.x = 3.0;
  angleMax.scale.y = 0.05;

  // Set orientation
  tf::Quaternion qMin = tf::createQuaternionFromYaw(viewMinMax[0]);
  tf::quaternionTFToMsg(qMin, angleMin.pose.orientation);
  
  tf::Quaternion qMax = tf::createQuaternionFromYaw(viewMinMax[1]);
  tf::quaternionTFToMsg(qMax, angleMax.pose.orientation);

  angleMin.color.r = 0.5;
  angleMin.color.g = 0.5;
  angleMin.color.b = 0.5;
  angleMin.color.a = 1;

  angleMax.color.r = 0.5;
  angleMax.color.g = 0.5;
  angleMax.color.b = 0.5;
  angleMax.color.a = 1;

  angleMin.lifetime = ros::Duration(20);
  angleMax.lifetime = ros::Duration(20);

  viewingInfo.markers.push_back(angleMin);
  viewingInfo.markers.push_back(angleMax);


  /*
   * Set common values for all Marker objects
   */ 
  for(int i=0;i<viewingInfo.markers.size();i++)
  {
    viewingInfo.markers[i].header.stamp = ros::Time::now();
    viewingInfo.markers[i].header.frame_id = global_frame;
    viewingInfo.markers[i].ns = "basic_shapes";
    viewingInfo.markers[i].id = 100000 + i;
    //////ROS_INFO("viewingInfo id: %i i: %i", viewingInfo.markers[i].id, i);
  }

  pub_rviz.publish(viewingInfo);
}


void odomCb(const nav_msgs::OdometryConstPtr msg)
{
  //////////ROS_INFO("In odomCb");
  //////////ROS_INFO("msg: Position: (%f,%f,%f)", msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

  // Need to create a MotionState from odom msg
  ramp_msgs::MotionState ms;

  if(!initOdom)
  {
    initialOdomState.positions.push_back(msg->pose.pose.position.x);
    initialOdomState.positions.push_back(msg->pose.pose.position.y);
    double theta = tf::getYaw(msg->pose.pose.orientation);
    initialOdomState.positions.push_back(theta);

    initOdom = true;
  }
  else
  {
    // Get new odometry changes
    double delta_x = (msg->pose.pose.position.x - initialOdomState.positions[0] - robotState.positions[0]);
    double delta_y = msg->pose.pose.position.y - initialOdomState.positions[1] - robotState.positions[1];
    double delta_th = util.findAngleBetweenAngles(initialOdomState.positions[2], tf::getYaw(msg->pose.pose.orientation));
    delta_th = util.findAngleBetweenAngles(robotState.positions[2], tf::getYaw(msg->pose.pose.orientation));

    // Apply changes
    ms = robotState;
    ms.positions[0] += delta_x;
    ms.positions[1] += delta_y;
    ms.positions[2] = util.displaceAngle(robotState.positions[2], delta_th);

    ms.positions[0] = msg->pose.pose.position.x;
    ms.positions[1] = msg->pose.pose.position.y;
    ms.positions[2] = util.displaceAngle(initialOdomState.positions[2], tf::getYaw(msg->pose.pose.orientation));

    //////////ROS_INFO("Calling setRobotPos with position (%f,%f,%f)", ms.positions[0], ms.positions[1], ms.positions[2]);

    // And then call setRobotPos
    setRobotPos(ms);
  }
}

void robotUpdateCb(const ramp_msgs::MotionStateConstPtr ms)
{
  ////ROS_INFO("In robotUpdateCb");
  //////////ROS_INFO("ms: %s", util.toString(*ms).c_str());
  setRobotPos(*ms);
  ////////ROS_INFO("Exiting robotUpdateCb");
}


bool checkViewingObstacle(Circle cir)
{
  //////////ROS_INFO("In checkViewingObstacle");
  
  if(robotState.positions.size() > 0)
  {
    //////////ROS_INFO("viewMinMax.size(): %i", (int)viewMinMax.size());
    //////////ROS_INFO("robotState.positions.size(): %i", (int)robotState.positions.size());
    
    std::vector<double> obMs;
    obMs.push_back(cir.center.x);
    obMs.push_back(cir.center.y);

    double angleToOb = util.findAngleFromAToB(robotState.positions, obMs);

    // Get difference between angleToOb and robot's orientation
    double deltaTheta = util.findDistanceBetweenAngles(robotState.positions[2], angleToOb);

    //////////ROS_INFO("[min,max]: [%f,%f] angleToOb: %f", viewMinMax[0], viewMinMax[1], angleToOb);
    //////////ROS_INFO("robotState.positions[2]: %f deltaTheta: %f fovAngle: %f", robotState.positions[2], deltaTheta, fovAngle);


    if(fabs(deltaTheta) < fovAngle/2.0)
    {
      //////////ROS_INFO("Returning true for deltaTheta");
      return true;
    }
  }

  //////////ROS_INFO("Returning false");
  return false;
}

void convertCircleToGlobal(const nav_msgs::OccupancyGrid& grid, Circle& cir)
{
  double x_origin = grid.info.origin.position.x;
  double y_origin = grid.info.origin.position.y;
  ROS_INFO("x_origin: %f, y_origin: %f resolution: %f", x_origin, y_origin, grid.info.resolution);

  cir.center.x = (cir.center.x * grid.info.resolution) + x_origin;
  cir.center.y = (cir.center.y * grid.info.resolution) + y_origin;
  cir.radius *= grid.info.resolution;
}

void convertGroups(const nav_msgs::OccupancyGrid& grid, std::vector<CircleGroup>& groups)
{
 
  // Convert fit circles in outer loop
  for(int i=0;i<groups.size();i++)
  {
    //ROS_INFO("Fit circle for cirGroup %i center: (%f,%f) radius: %f", i, groups[i].fitCir.center.x, groups[i].fitCir.center.y, groups[i].fitCir.radius);
    convertCircleToGlobal(grid, groups[i].fitCir);
    //ROS_INFO("New point: (%f,%f) New radius: %f", groups[i].fitCir.center.x, groups[i].fitCir.center.y, groups[i].fitCir.radius);

    // Convert packed circles in inner loop
    for(int j=0;j<groups[i].packedCirs.size();j++)
    {
      ROS_INFO("Packed circle %i center: (%f,%f) radius: %f ", j, groups[i].packedCirs[j].center.x, groups[i].packedCirs[j].center.y, groups[i].packedCirs[j].radius);
      convertCircleToGlobal(grid, groups[i].packedCirs[j]);
      ROS_INFO("New Point: (%f,%f) New Radius: %f ", groups[i].packedCirs[j].center.x, groups[i].packedCirs[j].center.y, groups[i].packedCirs[j].radius);
    }
  }
}


void computeOrientations()
{
  // Check the costmap frequency
  if(num_costmaps % num_costmap_freq_theta == 0 || cirGroups.size() > prev_valid_cirs.size())
  {
    // Average the theta values
    std::vector<double> thetas = predictTheta();
    for(int i=0;i<cir_obs.size();i++)
    {
      //////////ROS_INFO("Obstacle %i", i);
      //////////ROS_INFO("theta[%i]: %f", i, thetas[i]);
      cir_obs[i]->prevTheta.push_back(thetas[i]);
      if(cir_obs[i]->prevTheta.size() > num_theta_count)
      {
        //////////ROS_INFO("Removing %f", cir_obs[i]->prevTheta[0]);
        cir_obs[i]->prevTheta.erase( cir_obs[i]->prevTheta.begin(), cir_obs[i]->prevTheta.begin()+1 );
      }

      double theta = cir_obs[i]->prevTheta[0];
      for(int j=1;j<cir_obs[i]->prevTheta.size();j++)
      {
        theta = util.findAngleBetweenAngles(theta, cir_obs[i]->prevTheta[j]);
        //////////ROS_INFO("j: %i theta: %f next theta: %f result: %f", j, theta, cir_obs[i]->prevTheta[j], util.findAngleBetweenAngles(theta, cir_obs[i]->prevTheta[j]));
      }

      // Set new theta value
      cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] = theta;
    }
  } // end if costmap is considered for predicting theta values
}


/*
 * Creates an Obstacle object for each element in cirGroups and pushes them onto the ObstacleList
 */
void populateObstacleList(const std::vector<Velocity>& velocities)
{
  //////////ROS_INFO("In populateObstacleList");
  
  obs.clear();
  dynamicObsList.obstacles.clear();
  for(int i=0;i<cir_obs.size();i++)
  {
    //////////ROS_INFO("Creating obstacle, prevCirs.size(): %i prevTheta.size(): %i", (int)cir_obs[i]->prevCirs.size(), (int)cir_obs[i]->prevTheta.size());
    //////////ROS_INFO("Circle Group %i: %s", i, util.toString(cir_obs[i]->cirGroup).c_str());
    Obstacle o;

    // Update values
    float theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : initial_theta;
    o.update(cir_obs[i]->cirGroup, velocities[i], theta);
    
    obs.push_back(o);
    dynamicObsList.obstacles.push_back(o.msg_);
    //ROS_INFO("ob %i position: (%f,%f)", i, obs[i].msg_.ob_ms.positions[0], obs[i].msg_.ob_ms.positions[1]);
  }
  
  //////////ROS_INFO("Exiting populateObstacleList");
}


/*
 * Calls predictVelocities to get the latest speed values and then averages over the last N times
 * Sets the list of velocities in result vector
 */
void computeVelocities(const std::vector<CircleMatch> cm, const ros::Duration d_elapsed, std::vector<Velocity>& result)
{
  //ROS_INFO("In computeVelocities");
  result = predictVelocities(cm, d_elapsed);

  // Average the velocities
  for(int i=0;i<cir_obs.size();i++)
  {
    //ROS_INFO("i: %i cir_obs.size(): %i", i, (int)cir_obs.size());
    //ROS_INFO("result.size(): %i cir_obs[%i]->prevTheta.size(): %i", (int)result.size(), i, (int)cir_obs.size());

    cir_obs[i]->vels.push_back(result[i]);
    if(cir_obs[i]->vels.size() > num_velocity_count)
    {
      cir_obs[i]->vels.erase( cir_obs[i]->vels.begin(), cir_obs[i]->vels.begin()+1 );
    }

    float v=0;
    for(int n=0;n<cir_obs[i]->vels.size();n++)
    {
      //////////ROS_INFO("n: %i Adding %f", n, cir_obs[i]->vels[n].v);
      v += cir_obs[i]->vels[n].v;
    }
    v /= cir_obs[i]->vels.size();
    //////////ROS_INFO("Averaged v: %f", v);

    float theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : initial_theta;
    float vx = v*cos(theta);
    float vy = v*sin(theta);

    // Set values, check for static obstacles
    if(result[i].v < static_v_threshold || !cir_obs[i]->moving)
    {
      result[i].v   = 0;
      result[i].vx  = 0;
      result[i].vy  = 0;

      // Increment static count
      cir_obs[i]->static_count++;
      if(cir_obs[i]->static_count > ob_not_moving_count)
      {
        ////ROS_INFO("Setting moving = false, static_count: %i", cir_obs[i]->static_count);
        cir_obs[i]->moving = false;
      }
    }
    else
    {
      result[i].v   = v;
      result[i].vx  = vx;
      result[i].vy  = vy;

      // Reset static_count
      cir_obs[i]->static_count = 0;
    }

    // Set updated velocity value
    cir_obs[i]->vels[cir_obs[i]->vels.size()-1] = result[i];
    cir_obs[i]->vel = result[i];
    //////////ROS_INFO("Velocity %i: v: %f vx: %f vy: %f w: %f", i, velocities[i].v, velocities[i].vx, velocities[i].vy, velocities[i].w);
  } // end for

  ////ROS_INFO("Exiting computeVelocities");
}


void staticMapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  ROS_INFO("In staticMapCb");
  
  if(staticObs.size() == 0)
  {
    staticMap = *grid;
    ros::Time tStart = ros::Time::now();
    CirclePacker c(grid); // (If using modified costmap)
    
    c.setStaticMap(grid);
    c.staticMap_ = *grid;

    // Pixels seem to be correct
    staticObs = c.getGroupsForStaticMap();
    

    convertGroups(staticMap, staticObs);

    ROS_INFO("staticObs.size(): %i", (int)staticObs.size());
    ros::Duration d = ros::Time::now() - tStart;
    for(int i=0;i<staticObs.size();i++)
    {
      ROS_INFO("static ob %i - Center: (%f,%f) Radius: %f packedCirs.size(): %i", i, staticObs[i].fitCir.center.x, staticObs[i].fitCir.center.y, staticObs[i].fitCir.radius, (int)staticObs[i].packedCirs.size());
      for(int j=0;j<staticObs[i].packedCirs.size();j++)
      {
        ROS_INFO("  Packed circle %i: %f,%f Radius=%f", j, staticObs[i].packedCirs[j].center.x, staticObs[i].packedCirs[j].center.y, staticObs[i].packedCirs[j].radius);
      }
      Obstacle o(staticObs[i]);
      staticObsList.obstacles.push_back(o.msg_);
    }
    //ROS_INFO("Elapsed time: %f", d.toSec());
  }
  else
  {
    ROS_INFO("Already got static map obstacles");
  }
  gotPersistent = true;
}

void costmapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  /*ROS_INFO("**************************************************");
  ROS_INFO("In costmapCb");
  ROS_INFO("**************************************************");*/
  ros::Duration d_elapsed = ros::Time::now() - t_last_costmap;
  t_last_costmap = ros::Time::now();
  high_resolution_clock::time_point tStart = high_resolution_clock::now();
  ////ROS_INFO("grid (w,h): (%i,%i) origin: (%f,%f)", grid->info.width, grid->info.height, grid->info.origin.position.x, grid->info.origin.position.y);

  // Consolidate this occupancy grid with prev ones
  nav_msgs::OccupancyGrid accumulated_grid;

  if(cropMap)
  {
    /*
     * Only use half of the costmap since the kinect can only see in front of the robot
     */
    
    // Update global grid
    nav_msgs::OccupancyGrid cropped;
    cropCostmap(grid, cropped);

    transformCostmap(cropped);
    pub_global_costmap.publish(global_costmap);

    double grid_resolution = grid->info.resolution; 
    
    global_grid = cropped;
    ////ROS_INFO("global grid (w,h): (%i,%i)", global_grid.info.width, global_grid.info.height);
    

    //////////ROS_INFO("Resolution: width: %i height: %i", grid->info.width, grid->info.height);
    accumulateCostmaps(global_costmap, prev_grids, accumulated_grid);
    
    // Set global_grid 
    global_grid = global_costmap;
    //////////ROS_INFO("global grid (w,h): (%i,%i)", global_grid.info.width, global_grid.info.height);
    //////////ROS_INFO("global costmap (w,h): (%i,%i)", global_costmap.info.width, global_costmap.info.height);
    
    //////////ROS_INFO("Finished getting consolidated_grid");
    
    // Push this grid onto prev_grids
    //prev_grids.push_back(global_grid);
    prev_grids.push_back(global_costmap);
    if(prev_grids.size() > num_costmaps_accumulate)
    {
      prev_grids.erase(prev_grids.begin(), prev_grids.begin()+1);
    }

    // Publish the modified costmap(s)
    //pub_half_costmap.publish(half);
    pub_cons_costmap.publish(accumulated_grid);
    //////ROS_INFO("accumulated grid (w,h): (%i,%i)", accumulated_grid.info.width, accumulated_grid.info.height);
  }
  else
  {
    //ROS_INFO("In else");
    accumulated_grid = *grid;
    global_grid = *grid;
    
    // ***
    transformCostmap(global_grid);
    pub_global_costmap.publish(global_costmap);

    double grid_resolution = grid->info.resolution; 
    
    global_grid = *grid;
    ////ROS_INFO("global grid (w,h): (%i,%i)", global_grid.info.width, global_grid.info.height);
    

    //////////ROS_INFO("Resolution: width: %i height: %i", grid->info.width, grid->info.height);
    accumulateCostmaps(global_costmap, prev_grids, accumulated_grid);
    //ROS_INFO("After accumulateCostmaps");
    
    // Set global_grid 
    global_grid = global_costmap;
    //////////ROS_INFO("global grid (w,h): (%i,%i)", global_grid.info.width, global_grid.info.height);
    //////////ROS_INFO("global costmap (w,h): (%i,%i)", global_costmap.info.width, global_costmap.info.height);
    
    //////////ROS_INFO("Finished getting consolidated_grid");
    
    // Push this grid onto prev_grids
    //prev_grids.push_back(global_grid);
    prev_grids.push_back(global_costmap);
    if(prev_grids.size() > num_costmaps_accumulate)
    {
      prev_grids.erase(prev_grids.begin(), prev_grids.begin()+1);
    }

    // Publish the modified costmap(s)
    //pub_half_costmap.publish(half);

    // ***

    pub_cons_costmap.publish(accumulated_grid);
  }

  // Make a pointer for the modified costmap
  boost::shared_ptr<nav_msgs::OccupancyGrid> cg_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(accumulated_grid);
  //boost::shared_ptr<nav_msgs::OccupancyGrid> cg_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(half);

  //////////ROS_INFO("consolidated_grid.data.size(): %i", (int)consolidated_grid.data.size());
  ////////ROS_INFO("cg_ptr->data.size(): %i", (int)cg_ptr->data.size());



  /*
   ********************************************
   * Finding circles on latest costmap
   ********************************************
   */

  CirclePacker c(cg_ptr); // (If using modified costmap)
  //CirclePacker c(grid); // (If not modifying costmap)
  
  // Do they have the same obstacle indices?
  //std::vector<Circle> cirs = c.goMyBlobs();
  cirGroups.clear();
  cirGroups = c.getGroups(staticObs, global_grid.info.origin.position.x, global_grid.info.origin.position.y, global_grid.info.resolution);
  polygonLines = c.polygonMarker_;
  pLines = c.pMarkers_;
  cLines = c.cMarkers_;

  //ROS_INFO("Finished getting Circle Groups");
  

  ROS_INFO("cirGroups.size(): %i", (int)cirGroups.size());
  for(int i=0;i<cirGroups.size();i++)
  {
    ROS_INFO("cirGroups[%i].fitCir.size: %f", i, cirGroups[i].fitCir.radius);
    ROS_INFO("cirGroups[%i].packedCirs.size(): %i", i, (int)cirGroups[i].packedCirs.size());
  }




  /*
   ********************************************
   * Done finding circles on latest costmap
   ********************************************
   */

  /*
   * Convert centers and radii to the global frame
   */ 
  //convertGroups(global_grid, cirGroups);
  /*for(int i=0;i<cirGroups.size();i++)
  {
    ////ROS_INFO("cirGroups[%i].fitCir.size: %f", i, cirGroups[i].fitCir.radius);
  }*/



   /*
    * Check if obstacles are in viewing angle
    */
  if(remove_outside_fov)
  {
    int i=0;
    while(i<cirGroups.size())
    {
      if(!checkViewingObstacle(cirGroups[i].fitCir))
      {
        cirGroups.erase(cirGroups.begin()+i, cirGroups.begin()+i+1);
        i--;
      }

      i++;
    }
  }
  //ROS_INFO("After checking viewing angle, cirGroups.size(): %i", (int)cirGroups.size());
 

  /*
   * Data association
   * Do the data associate before kf updates so that 
   * 1) cir_obs.cir is set before doing update
   * 2) the measurement for each cir_ob is correct before doing update
   */
  std::vector<CircleMatch> cm = dataAssociation(cirGroups);
  //ROS_INFO("Done checking data association");


  
  /*
   * Call the Kalman filter
   */
   std::vector<CircleGroup> circles_current = updateKalmanFilters(cirGroups, cm);
   //ROS_INFO("After updateKalmanFilters");

  

   
  /*
   * Circle positions are finalized at this point
   */




   /*
    * Predict orientations
    */
   computeOrientations();
   //ROS_INFO("After computeOrientations");
  
  
  /*
   * Predict velocities
   */
   std::vector<Velocity> velocities;
   computeVelocities(cm, d_elapsed, velocities);
   prev_velocities.push_back(velocities);
   //ROS_INFO("After computeVelocities");




  // Get attachments
  /*attachs.clear();
  if(cirs.size() > 0)
  {
    c.detectAttachedCircles(cir_obs, attachs);  
  }*/
  /*for(int i=0;i<attachs.size();i++)
  {
    ////////ROS_INFO("Attachment %i:", i);
    for(int j=0;j<attachs[i].cirs.size();j++)
    {
      ////////ROS_INFO("%i", attachs[i].cirs[j]);
    }
  }*/

  /*
   * Handle attachments
   */
  /*for(int i=0;i<attachs.size();i++)
  {
    //////////ROS_INFO("Attachment %i", i);
    // Get max speed among attached obstacles
    int i_max_speed=0;
    float speed_average = 0;
    for(int j=0;j<attachs[i].cirs.size();j++)
    {
      int i_cir = attachs[i].cirs[j];
      speed_average += cir_obs[i_cir]->vel.v;
    }
    speed_average /= attachs[i].cirs.size();

    double theta = cir_obs[i_max_speed]->prevTheta[cir_obs[i_max_speed]->prevTheta.size()-1];
    //////////ROS_INFO("attachs.size(): %i i: %i", (int)attachs.size(), i);

    // Based on max speed, set all circles speeds and thetas in attachment
    for(int j=0;j<attachs[i].cirs.size();j++)
    {
      //////////ROS_INFO("j: %i attachs[%i].cirs.size(): %i", j, i, (int)attachs[i].cirs.size());
      int i_cir   = attachs[i].cirs[j];
      int i_theta = cir_obs[i_cir]->prevTheta.size()-1;
      //////////ROS_INFO("i_cir: %i i_theta: %i cir_obs.size(): %i velocities.size(): %i", i_cir, i_theta, (int)cir_obs.size(), (int)velocities.size());
      cir_obs[ i_cir ]->vel.v = speed_average;
      //cir_obs[ i_cir ]->theta = theta;

      if(cir_obs[i_cir]->prevTheta.size() > 0)
      {
        cir_obs[ i_cir ]->prevTheta[i_theta] = theta;
      }
      else
      {
        cir_obs[ i_cir ]->prevTheta.push_back(theta);
      }
      
      velocities[ i_cir ].v = speed_average;
    } // end inner for
  } // end outer for*/

  
  /*
   * Set previous circles
   */
  for(int i=0;i<cir_obs.size();i++)
  {
    cir_obs[i]->prevCirs.push_back(cir_obs[i]->cirGroup.fitCir);
  }

  // Set prev_cirs variable for data association
  prev_valid_cirs = circles_current;
 


  /*
   *  After finding velocities, populate Obstacle list
   */  
  populateObstacleList(velocities);



  // Record duration data
  duration<double> time_span = duration_cast<microseconds>(high_resolution_clock::now()-tStart);
  durs.push_back( time_span.count() );
  
  if(use_static_map == true && gotPersistent == false)
  {
    gotPersistent = true;
  }


  ////ROS_INFO("Duration: %f", time_span.count());
  num_costmaps++;
  /*ROS_INFO("**************************************************");
  ROS_INFO("Exiting costmapCb");
  ROS_INFO("**************************************************");*/
}

void writeData()
{
  // General data files
  std::string directory = ros::package::getPath("ramp_sensing");
  
  std::ofstream f_durs;
  f_durs.open(directory+"/durations.txt");

  for(int i=0;i<durs.size();i++)
  {
    f_durs<<"\n"<<durs[i];
  }

  f_durs.close();
}

void deallocate()
{

  // Also, free up some memory
  if(meas_pdf)
  {
    delete meas_pdf;
    meas_pdf = 0;
  }
  printf("\nFreed meas_pdf\n");
  if(sys_pdf)
  {
    delete sys_pdf;
    sys_pdf = 0;
  }
  printf("\nFreed meas_model\n");

  for(int i=0;i<cir_obs.size();i++)
  {
    delete cir_obs[i];
    cir_obs[i] = 0;
  }

  printf("\nDone freeing memory\n");
}


void reportPredictedVelocity(int sig)
{
  timer_markers.stop();


  printf("\npredicted_velocities.size(): %i", (int)predicted_velocities.size());
  if(predicted_velocities.size() > 0)
  {
    double min_v=predicted_velocities.at(0).v, max_v = min_v, average_v=min_v;
    double min_w=predicted_velocities.at(0).w, max_w = min_w, average_w=min_w;
    int count_v=0, count_w=0;
    for(int i=1;i<predicted_velocities.size();i++)
    {
      if(predicted_velocities[i].v < min_v)
      {
        min_v = predicted_velocities[i].v;
      }
      if(predicted_velocities[i].v > max_v)
      {
        max_v = predicted_velocities[i].v;
      }

      if(predicted_velocities[i].v > 0)
      {
        //////////ROS_INFO("Adding %f", predicted_velocities[i].v);
        average_v += predicted_velocities[i].v;
        count_v++;
      }
      
      if(predicted_velocities[i].w < min_w)
      {
        min_w = predicted_velocities[i].w;
      }
      if(predicted_velocities[i].w > max_w)
      {
        max_w = predicted_velocities[i].w;
      }

      if(predicted_velocities[i].w > 0)
      {
        //////////ROS_INFO("Adding %f", predicted_velocities[i].w);
        average_w += predicted_velocities[i].w;
        count_w++;
      }
    }
    average_v /= count_v;
    average_w /= count_w;

    printf("\nPredicted Velocities range=[%f,%f], average: %f\n", min_v, max_v, average_v);
    printf("\nPredicted Velocities range=[%f,%f], average: %f\n", min_w, max_w, average_w);
  }

  //////////ROS_INFO("Average differences in circle detection");
  double d=0;
  int count=0;
  for(int i=0;i<d_avg_values.size();i++)
  {
    //////////ROS_INFO("d_avg_values[%i]: %f", i, d_avg_values[i]);
    if(!std::isnan(d_avg_values[i]))
    {
      d+=d_avg_values[i];
      count++;
    }
  }
  d = count > 0 ? d/count : 0;
  //////////ROS_INFO("Final average difference: %f", d);

  /*for(int i=0;i<cirs_pos.size();i++)
  {
    //////////ROS_INFO("cir_pos[%i]: (%f, %f)", i, cirs_pos[i].center.x, cirs_pos[i].center.y);
  }*/

  deallocate();
  writeData();


  // Add this line to keep the node from hanging
  // Not sure why it hangs without this line
  ros::shutdown();
}


// Make this node load data rather than load_occ_map so that it doesn't have to wait
// Waiting can take a while and if we put them in a launch file we can't control the launch order
// so it may miss the load_occ_map publish entirely
void loadPersistentGrid()
{
  std::string filename = "persistent_grid.txt";
  std::fstream fgrid;

  fgrid.open(filename, std::ios::in);
  if(fgrid.is_open() == false)
  {
    //ROS_ERROR("Cannot open persistent grid file");
  }
  else
  {
    
  }
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ramp_sensing");
  ros::NodeHandle handle;
  
  //count = 0;

  //Get parameters
  
  /*std::string other_robot_odom;
  handle.getParam("ramp_sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;*/


  loadParameters(handle);
  //ROS_INFO("use_odom_topics: %s", use_odom_topics ? "True" : "False");
    
  std::vector< ros::Subscriber > subs_obs;

  if(use_odom_topics)
  {
    ROS_INFO("In use_odom_topics");

    // Get the rosparam value
    if(handle.hasParam("/ramp/obstacle_odoms"))
    {
      handle.getParam("/ramp/obstacle_odoms", ob_odoms);
      for(int i=0;i<ob_odoms.size();i++)
      {
        topic_index_map[ob_odoms.at(i)] = i;
      }
    }
    else
    {
      //////////ROS_ERROR("ramp_sensing: Could not find obstacle_topics rosparam!");
    }

    // Load obstacle transforms to global
    loadObstacleTF();

    // Create subscribers
    //std::vector< ros::Subscriber > subs_obs;
    for(uint8_t i=0;i<ob_odoms.size();i++)
    {
      Obstacle temp;
      temp.T_w_init_ = ob_tfs[i];
      obs.push_back(temp);
      dynamicObsList.obstacles.push_back(temp.msg_);

      ros::Subscriber sub_ob = handle.subscribe<nav_msgs::Odometry>(ob_odoms.at(i), 1, boost::bind(updateOtherRobotCb, _1, ob_odoms.at(i)));
      subs_obs.push_back(sub_ob);
    } // end for*/
  }

  else
  {
    initGlobalMap();


    // Initialize the Kalman Filter
    init_linear_system_model();
    init_measurement_model();
    init_prior_model();

    //ROS_INFO("In Duration");
    ros::Duration d(2.5);

    /*tf::TransformListener listener;
    if(listener.waitForTransform(global_frame, robot_base_frame, ros::Time(0), d))
    {
      listener.lookupTransform(global_frame, robot_base_frame, ros::Time(0), tf_base_to_global);
      //listener.lookupTransform(robot_base_frame, global_frame, ros::Time(0), tf_base_to_global);
      ////////ROS_INFO("Base to global tf: translate: (%f, %f) rotation: %f", tf_base_to_global.getOrigin().getX(), tf_base_to_global.getOrigin().getX(), tf_base_to_global.getRotation().getAngle());
    }
    else
    {
      ////////ROS_ERROR("Could not find global tf");
    }*/

    // Before instantiating Subscribers, check if we are using a persistent grid
    if(use_static_map)
    {
      handle.setParam("/ramp/sensing_ready", false);
      ROS_INFO("Rosparam staticMap is true so this node will wait until a nav_msgs/OccupancyGrid is published on '/map'");
      ros::Subscriber sub_staticMap = handle.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &staticMapCb);

      ros::Rate r(100);
      while(ros::ok() && gotPersistent == false) {ros::spinOnce(); r.sleep();ROS_INFO("In while");}
      
      ROS_INFO("Got grid!");
    }
  }
  
  ROS_INFO("Sensing module setting /ramp/sensing_ready to true");
  handle.setParam("/ramp/sensing_ready", true);

  if(use_hilbert_map)
  {
    ROS_INFO("Sensing module is waiting for planner to start...");
    ros::Rate rWaiting(1000);
    while(planner_started == false)
    {
      handle.getParam("ramp/preplanning_cycles_done", planner_started);
      ros::spinOnce();
      rWaiting.sleep();
    }
  }

  // Subscribers
  ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapCb);
  ros::Subscriber sub_robot_update = handle.subscribe<ramp_msgs::MotionState>("/updateAfterTf", 1, &robotUpdateCb);
  //ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/hilbert_map_grid", 1, &costmapCb);
  //ros::Subscriber sub_odom = handle.subscribe<nav_msgs::Odometry>("/odom", 1, &odomCb);
  //ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/consolidated_costmap", 1, &costmapCb);

  // Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 2);
  pub_cons_costmap = handle.advertise<nav_msgs::OccupancyGrid>("accumulated_costmap", 2);
  pub_half_costmap = handle.advertise<nav_msgs::OccupancyGrid>("half_costmap", 2);
  pub_global_costmap = handle.advertise<nav_msgs::OccupancyGrid>("global_costmap", 2);

  // Timers
  ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
  timer_markers = handle.createTimer(ros::Duration(1.f/10.f), publishMarkers);

 
  // Set function to run at shutdown
  signal(SIGINT, reportPredictedVelocity);
 
  ros::Duration dd(0.5);
  dd.sleep();

  // Set initial robot position
  // start vector needs to be set (usually it is obtained from rosparam)
  ramp_msgs::MotionState ms;
  ms.positions.push_back(start[0]);
  ms.positions.push_back(start[1]);
  ms.positions.push_back(start[2]);
  setRobotPos(ms);

  printf("\nSensing node Spinning\n");

  /*ros::AsyncSpinner spinner(8);
  printf("\nWaiting for requests...\n");
  spinner.start();
  ros::waitForShutdown();*/

  ros::spin();

  printf("\nExiting normally\n");
  return 0;
}
