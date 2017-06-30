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
#include <visualization_msgs/MarkerArray.h>
#include "utility.h"
#include "circle_filter.h"


#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/pdf/analyticconditionalgaussian.h>


std::vector<CircleOb*> cir_obs;
std::vector<Circle> cirs_pos;

Utility util;
double rate=10;
ros::Publisher pub_obj, pub_rviz, pub_cons_costmap, pub_half_costmap;
std::vector< Obstacle> obs;
ramp_msgs::ObstacleList list;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;
nav_msgs::OccupancyGrid global_grid;
std::string global_frame;

double radius;
std::vector<double> dof_min;
std::vector<double> dof_max;
std::vector<ramp_msgs::Range> ranges;


std::vector<tf::Transform> ob_tfs;

std::vector<Circle> prev_cirs;

// prev_velocities[cycle][circle]
std::vector< std::vector<Velocity> > prev_velocities;

ros::Time t_last_costmap;

int num_costmaps=0;


std::vector<nav_msgs::OccupancyGrid> prev_grids;

std::vector<Circle> prev_valid_cirs;


// Vector to store velocities to report data after execution
std::vector<Velocity> predicted_velocities;

ros::Timer timer_markers;

double coll_radius = 0.25;

std::vector<double> d_avg_values;

double dist_threshold = 0.5;
double radius_threshold = 0.5;

int num_costmaps_accumulate = 5;
int num_velocity_count      = 3;
int num_theta_count         = 1;
int num_costmap_freq_theta  = 10;

double initial_theta        = PI;

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
double MU_MEAS_NOISE = 0.0005;
double SIGMA_MEAS_NOISE = 0.0005;

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
    if(i == 0 || i == 1)
    {
      temp.min += radius;
      temp.max -= radius;
    }
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
    handle.getParam("costmap_node/costmap/resolution", costmap_res);

    ////ROS_INFO("Got costmap parameters. w: %i h: %i x: %f y: %f res: %f", costmap_width, costmap_height, costmap_origin_x, costmap_origin_y, costmap_res);
  }

  // Get the radius of the robot
  if(handle.hasParam("robot_info/radius")) 
  {
    handle.getParam("robot_info/radius", radius);
  }
  else 
  {
    ////ROS_ERROR("Did not find parameter robot_info/radius");
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
    ////ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }

  if(handle.hasParam("/ramp/global_frame"))
  {
    handle.getParam("/ramp/global_frame", global_frame);
  }
  else
  {
    ////ROS_ERROR("Did not find rosparam /ramp/global_frame");
  }

}



void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ////ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ////ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ////ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(std::stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ////ROS_INFO("Last token: %s", line.c_str());

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
  ////ROS_INFO("In init_measurement_model");

  MatrixWrapper::Matrix H(STATE_SIZE,STATE_SIZE);
  H = 0.0;

  // Set x and y
  H(1,1) = 1;
  H(2,2) = 1;

  ////ROS_INFO("Setting mu");

  BFL::ColumnVector meas_noise_mu(STATE_SIZE);
  meas_noise_mu(1) = MU_MEAS_NOISE;
  meas_noise_mu(2) = MU_MEAS_NOISE;
  meas_noise_mu(3) = 0;
  meas_noise_mu(4) = 0;

  ////ROS_INFO("Setting cov");

  MatrixWrapper::SymmetricMatrix meas_noise_cov(STATE_SIZE);
  meas_noise_cov = 0.0;
  for(int i=1;i<=STATE_SIZE;i++)
  {
    meas_noise_cov(i,i) = SIGMA_MEAS_NOISE;
  }

  ////ROS_INFO("Setting measurement_uncertainty");

  // Make the Gaussian
  BFL::Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);

  ////ROS_INFO("Setting meas_pdf");

  // Make the pdf
  meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty);
}


void init_prior_model()
{
  ////ROS_INFO("In init_prior_model");

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
  ////ROS_INFO("In init_linear_system_model");

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

  ////ROS_INFO("Initializing the linear system uncertainty"); 

  BFL::Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

  sys_pdf = new BFL::LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
}



/** Get the other robot's current odometry information and update the obstacle info */
void updateOtherRobotCb(const nav_msgs::Odometry::ConstPtr& o, const std::string& topic) 
{
  //////ROS_INFO("In updateOtherRobotCb");
  //////ROS_INFO("topic: %s", topic.c_str());
  int index = topic_index_map[topic];
  //////ROS_INFO("index: %i", index);

  if(obs.size() < index+1)
  {
    //////ROS_INFO("In if obs.size() < index");
    Obstacle temp(*o);
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);
  }
  else
  {
    //////ROS_INFO("In else");
    obs.at(index).update(*o);
    list.obstacles.at(index) = obs.at(index).msg_;
  }
} //End updateOtherRobotCb



std::vector<visualization_msgs::Marker> convertObsToMarkers()
{
  std::vector<visualization_msgs::Marker> result;
  
  if(global_grid.info.width != 0 && global_grid.info.height != 0)
  {
    double x_origin = global_grid.info.origin.position.x;
    double y_origin = global_grid.info.origin.position.y;
    ////ROS_INFO("x_origin: %f y_origin: %f", x_origin, y_origin);

    x_origin /= global_grid.info.resolution;
    y_origin /= global_grid.info.resolution;

    // For each CircleOb, make a Marker
    for(int i=0;i<cir_obs.size();i++)
    {
      ////ROS_INFO("i: %i obs.size(): %i", i, (int)obs.size());
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = global_frame;
      //marker.header.frame_id = "/costmap";
      marker.ns = "basic_shapes";
      marker.id = i;
      
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      // Set x and y
      double x = cir_obs[i]->cir.center.x;
      double y = cir_obs[i]->cir.center.y;
      //double x = (cir_obs[i]->cir.center.x + x_origin) * global_grid.info.resolution;
      //double y = (cir_obs[i]->cir.center.y + y_origin) * global_grid.info.resolution;

      ////ROS_INFO("cir_obs[%i]->cir.center.x: %f cir_obs[%i]->cir.center.y: %f cir_obs[%i]->cir.radius: %f", i, cir_obs[i]->cir.center.x, i, cir_obs[i]->cir.center.y, i, cir_obs[i]->cir.radius);
      ////ROS_INFO("(x,y): (%f,%f) x_origin: %f y_origin: %f", x, y, x_origin, y_origin);

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      double radius = cir_obs[i]->cir.radius;
      ////ROS_INFO("Marker info x: %f y: %f radius: %f", x, y, radius);
      
      /*obs[i].cir_.center.x = x;
      obs[i].cir_.center.y = y;
      obs[i].cir_.radius = radius;*/
      
      // scale values are the diameter so use the radius*2
      marker.scale.x = radius*2.00f;
      marker.scale.y = radius*2.00f;
      marker.scale.z = 0.1;
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;
      marker.lifetime = ros::Duration(0.1);

      result.push_back(marker);
    }
  }
  return result;
}


/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  pub_obj.publish(list);
} //End sendList



void publishMarkers(const ros::TimerEvent& e)
{
  ////ROS_INFO("In publishMarkers");
  
  // Publish a single Marker
  visualization_msgs::MarkerArray result;

  
  std::vector<visualization_msgs::Marker> markers = convertObsToMarkers();

  ////ROS_INFO("Publishing %i markers", (int)markers.size());

  result.markers = markers;
  
  std::vector<visualization_msgs::Marker> texts;
  std::vector<visualization_msgs::Marker> arrows;

  ////ROS_INFO("markers.size(): %i", (int)markers.size());
  ////ROS_INFO("prev_velocities.size(): %i", (int)prev_velocities.size());

  /*
   * Make text to show the linear velocity value for each object
   * Make an arrow to show the direction of the linear velocity
   */
  for(int i=0;i<markers.size();i++)
  {
    ////ROS_INFO("Creating text and arrow for marker i: %i", i);
    ////ROS_INFO("prev_velocities.size(): %i", (int)prev_velocities.size());
    ////ROS_INFO("prev_velocities[%i].size(): %i", i, (int)prev_velocities[i].size());
    ////ROS_INFO("Marker %i position: (%f, %f) v: %f", i, markers[i].pose.position.x, markers[i].pose.position.y, prev_velocities[prev_velocities.size()-1][i].v);
    visualization_msgs::Marker text;
    visualization_msgs::Marker arrow;

    /*
     * Set members for text and arrow markers
     */
    text.header.stamp   = ros::Time::now();
    arrow.header.stamp  = ros::Time::now();

    text.id   = markers.size()+i;
    arrow.id  = markers.size()*(i+markers.size()+1);

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
    
    // Set text value
    // Should I use cir_obs->vel?
    if(prev_velocities[prev_velocities.size()-1].size() > 0)
    {
      text.text = std::to_string(prev_velocities[prev_velocities.size()-1][i].v);
      ////ROS_INFO("Velocity: %s", text.text.c_str());
    }

    // Set arrow points
    arrow.scale.x = prev_velocities[prev_velocities.size()-1][i].v < 0.25 ? 0.25 : prev_velocities[prev_velocities.size()-1][i].v;
    arrow.scale.y = 0.1;
    
    // Set poses
    text.pose = markers[i].pose;
    
    text.pose.position.z = 0.1;
    text.color.r = 0;
    text.color.g = 0;
    text.color.b = 1;
    text.color.a = 1;
    text.scale.z = 0.25;
    text.lifetime = ros::Duration(0.1);

    arrow.pose = text.pose;
    
    // Set the arrow orientation
    ////ROS_INFO("cir_obs.size(): %i prevTheta.size(): %i index: %i", (int)cir_obs.size(), (int)cir_obs[i]->prevTheta.size(), (int)cir_obs[i]->prevCirs.size()-1);

    double theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : 0;
    ////ROS_INFO("prevTheta.size(): %i theta: %f", (int)cir_obs[i]->prevTheta.size(), theta);
    tf::Quaternion q = tf::createQuaternionFromYaw(theta);
    tf::quaternionTFToMsg(q, arrow.pose.orientation);
    
   
    arrow.color.r = 1;
    arrow.color.g = 0;
    arrow.color.b = 0;
    arrow.color.a = 1;
    arrow.lifetime = ros::Duration(0.1);
    
    // Push onto texts 
    texts.push_back(text);
    arrows.push_back(arrow);
  }


  ////ROS_INFO("texts.size(): %i", (int)texts.size());

  result.markers.insert(std::end(result.markers), std::begin(texts), std::end(texts));  
  result.markers.insert(std::end(result.markers), std::begin(arrows), std::end(arrows));  

  // Create a text marker to show the size of cir_obs
  visualization_msgs::Marker text;
  text.header.stamp   = ros::Time::now();
  text.id   = result.markers.size()+1;
  //text.header.frame_id  = "/map";
  //text.header.frame_id  = "/costmap";
  text.header.frame_id  = global_frame;
  text.ns   = "basic_shapes";
  text.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action   = visualization_msgs::Marker::ADD;
  std::stringstream num_obs;
  num_obs<<"Number of obstacles: "<<(int)cir_obs.size();
  text.text = num_obs.str();

  // Set poses
  text.pose.position.x = 8;
  text.pose.position.y = 3.5;
  text.pose.position.z = 0.1;
  text.color.r = 0;
  text.color.g = 0;
  text.color.b = 0;
  text.color.a = 1;
  text.scale.z = 0.75;
  text.lifetime = ros::Duration(0.2);

  result.markers.push_back(text);
  

  ////ROS_INFO("result.markers.size(): %i", (int)result.markers.size());

  pub_rviz.publish(result);
  ////ROS_INFO("Exiting publishMarkers");
}


int getClosestPrev(Circle m, std::vector<Circle> N, std::vector<int> matched)
{
  //////ROS_INFO("In getClosestPrev");
  //////ROS_INFO("N.size(): %i", (int)N.size());
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

  //////ROS_INFO("center: [%f, %f]", center[0], center[1]);

  // If there are circles left to match
  if(N.size() > 0)
  {

    // Get center and dist used as the initial minimum
    std::vector<double> prev_center;
    prev_center.push_back(N.at(min_index).center.x);
    prev_center.push_back(N.at(min_index).center.y);

    double dist = util.positionDistance(center, prev_center);
    
    //////ROS_INFO("Prev center: [%f, %f]", prev_center[0], prev_center[1]);
    //////ROS_INFO("dist: %f", dist);

    // Go through remaining potential matches, find min dist
    for(int i=min_index;i<N.size();i++)
    {
      prev_center.at(0) =  N[i].center.x;
      prev_center.at(1) =  N[i].center.y;
      //////ROS_INFO("Prev center: [%f, %f] dist: %f", prev_center[0], prev_center[1], util.positionDistance(center, prev_center));

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



std::vector<Velocity> predictVelocities(const std::vector<CircleMatch> cm, const ros::Duration d_elapsed)
{
  ////ROS_INFO("In predictVelocities, d_elapsed: %f", d_elapsed.toSec());

  std::vector<Velocity> result;
  double grid_resolution=0.01;
  
  // For each circle obstacle,
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("CircleOb %i prevCirs.size(): %i", i, (int)cir_obs[i]->prevCirs.size());
    ////ROS_INFO("Current: (%f, %f)", cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);

    Velocity temp;

    // If prevCir is set 
    if(cir_obs[i]->prevCirs.size() > 0)
    {
      // Prev needs to be based on the global x,y
      int i_prev = cir_obs[i]->prevCirs.size()-1;
      ////ROS_INFO("Prev: (%f, %f)", cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y);
      double x_dist = cir_obs[i]->cir.center.x - cir_obs[i]->prevCirs[i_prev].center.x;
      double y_dist = cir_obs[i]->cir.center.y - cir_obs[i]->prevCirs[i_prev].center.y;
      //double dist = sqrt( pow(x_dist,2) + pow(y_dist,2) );

      double dist = util.positionDistance(cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y, cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);


      double theta = atan2(y_dist, x_dist);
      
      // Add on change in radius
      if(cm.size()>0)
      {
        dist += cm[0].delta_r;
        ////ROS_INFO("Adding delta r: %f to dist, new dist: %f", cm[0].delta_r, dist);
        
        // If so, then make theta be towards the robot
        //theta = PI;
      }

      double w = util.displaceAngle(theta, cir_obs[i]->prevTheta[i_prev]) / d_elapsed.toSec();

      double linear_v = (dist / d_elapsed.toSec());

      // Xdot and Ydot should be based on overall linear speed
      temp.vx = linear_v*cos(theta);
      temp.vy = linear_v*sin(theta);
      temp.v  = linear_v;
      temp.w  = w;
      ////ROS_INFO("dist: %f t: %f vx: %f vy: %f speed: %f theta: %f w: %f", dist, d_elapsed.toSec(), temp.vx, temp.vy, linear_v, theta, w);
      
      predicted_velocities.push_back(temp);
    }
    else
    {
      ////ROS_INFO("No previous circles");
      temp.vx = 0;
      temp.vy = 0;
      temp.v  = 0;
      temp.w  = 0;
    }

    // Push the circle's velocity onto the result
    result.push_back(temp);
  } // end for

  ////ROS_INFO("Exiting predictVelocities");
  return result;
}


std::vector<double> predictTheta()
{
  ////ROS_INFO("In predictTheta");
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
      /*////ROS_INFO("i_prev: %i", i_prev);
      ////ROS_INFO("Prev: (%f, %f) Current: (%f, %f)", cir_obs[i]->prevCirs[i_prev].center.x, cir_obs[i]->prevCirs[i_prev].center.y, cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);*/
      double x_dist = cir_obs[i]->cir.center.x - cir_obs[i]->prevCirs[i_prev].center.x;
      double y_dist = cir_obs[i]->cir.center.y - cir_obs[i]->prevCirs[i_prev].center.y;
      ////ROS_INFO("x_dist: %f y_dist: %f", x_dist, y_dist);

      theta = atan2(y_dist, x_dist);
    }
    
    result.push_back(theta); 
  } // end for each circle obstacle

  ////ROS_INFO("Exiting predictTheta");
  return result;
}


CircleOb* createCircleOb(Circle temp)
{
  CircleOb* result = new CircleOb;
  result->cir = temp;

  BFL::ColumnVector prior_mu(STATE_SIZE);
  prior_mu(1) = temp.center.x;
  prior_mu(2) = temp.center.y;
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

  ////ROS_INFO("Returning filter");
  return result;
}







void consolidateCostmaps(const nav_msgs::OccupancyGrid g1, const nav_msgs::OccupancyGrid g2, nav_msgs::OccupancyGrid& result)
{
  ////ROS_INFO("In consolidateCostmaps(OccupancyGrid, OccupancyGrid, OccupancyGrid)");
  ////ROS_INFO("g1.data.size(): %i", (int)g1.data.size());
  result = g1;
  ////ROS_INFO("g1.info.width: %i g1.info.height: %i", g1.info.width, g1.info.height);
  ////ROS_INFO("Before for loops, result.size(): %i", (int)result.data.size());
  for(int r=0;r<g1.info.width;r++)
  {
    int r_offset = g1.info.height*r;
    for(int c=0;c<g1.info.height;c++)
    {
      //////ROS_INFO("r_offset: %i c: %i r_offset+c: %i", r_offset, c, r_offset+c);
      result.data[r_offset + c] = g1.data[r_offset + c] | g2.data[r_offset + c];
    }
  }
  ////ROS_INFO("After for loops, result.size(): %i", (int)result.data.size());
}

void consolidateCostmaps(const nav_msgs::OccupancyGrid gi, const std::vector<nav_msgs::OccupancyGrid> prev_grids, nav_msgs::OccupancyGrid& result)
{
  ////ROS_INFO("In consolidateCostmaps(OccupancyGrid, vector<OccupancyGrid>, OccupancyGrid)");
  ////ROS_INFO("gi.size(): %i", (int)gi.data.size());
  
  if(prev_grids.size() == 0)
  {
    result = gi;
  }
  else
  {
    nav_msgs::OccupancyGrid temp = gi;

    for(int i=0;i<prev_grids.size();i++)
    {
      //////ROS_INFO("Consolidating with previous grid %i, prev_grid[%i].size(): %i", i, i, (int)prev_grids[i].data.size());
      consolidateCostmaps(temp, prev_grids[i], result);
      //////ROS_INFO("New result size: %i", (int)result.data.size());
      temp = result;
    }
  }
}


/*
 * Rename to matchCircles when done
 */
std::vector<CircleMatch> matchCircles(std::vector<Circle> cirs, std::vector<Circle> targets)
{
  ////ROS_INFO("In matchCircles");
  ////ROS_INFO("cirs.size(): %i targets.size(): %i", (int)cirs.size(), (int)targets.size());
  for(int i=0;i<cirs.size();i++)
  {
    ////ROS_INFO("Circle %i: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
  }
  for(int i=0;i<targets.size();i++)
  {
    ////ROS_INFO("Target %i: (%f,%f)", i, targets[i].center.x, targets[i].center.y);
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
      target_dists.push_back(util.positionDistance(cirs[j].center.x, cirs[j].center.y, targets[i].center.x, targets[i].center.y));

      // Create CircleMatch object
      CircleMatch temp;
      temp.i_cirs = j;
      temp.i_prevCir = i;
      temp.dist = util.positionDistance(cirs[j].center.x, cirs[j].center.y, targets[i].center.x, targets[i].center.y);
      temp.delta_r = fabs(cirs[j].radius - targets[i].radius);

      all_dists.push_back(temp);
    }

    dists.push_back(target_dists);
  } // end getting dist values

  ////ROS_INFO("all_dists.size(): %i", (int)all_dists.size());

  // Sort dist values
  std::sort(all_dists.begin(), all_dists.end(), util.compareCircleMatches);

  // Print all potential matches
  for(int i=0;i<all_dists.size();i++)
  {
    ////ROS_INFO("all_dists %i: i_cirs: %i targets: %i dist: %f delta_r: %f", i, all_dists[i].i_cirs, all_dists[i].i_prevCir, all_dists[i].dist, all_dists[i].delta_r);
  }
    
  int i=0;
  while(i < all_dists.size())
  {
    ////ROS_INFO("all_dists %i: i_cirs: %i targets: %i dist: %f delta_r: %f", i, all_dists[i].i_cirs, all_dists[i].i_prevCir, all_dists[i].dist, all_dists[i].delta_r);

    // Check if this is a legitimate match based on dist and radius change
    if(all_dists[i].dist < dist_threshold && all_dists[i].delta_r < radius_threshold)
    {
      ////ROS_INFO("Legitimate match");

      // Now check that the target or circle hasn't been matched already
      bool prev_matched = false;
      for(int r=0;r<result.size();r++)
      {
        ////ROS_INFO("r: %i result[%i].i_cirs: %i result[%i]: %i", r, r, result[r].i_cirs, r, result[r].i_cirs);
        if(result[r].i_cirs == all_dists[i].i_cirs || result[r].i_prevCir == all_dists[i].i_prevCir)
        {
          ////ROS_INFO("Previously matched!");
          prev_matched = true;
          break;
        }
      } // end for

      if(!prev_matched)
      {
        ////ROS_INFO("Not previously matched!");
        result.push_back(all_dists[i]);
        all_dists.erase(all_dists.begin()+i);
        i--;
      }
    } // end if legitimate match
    else
    {
      ////ROS_INFO("Match not legitimate");
    }

    i++;
  } // end for all_dists values


  ////ROS_INFO("Exiting matchCircles");
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
      ////ROS_INFO("matched_tar[%i]: %i", i, matched_tar[i]);
      ////ROS_INFO("Deleting filter at index %i!", index_cir_obs);
      delete cir_obs[index_cir_obs];
      cir_obs.erase(cir_obs.begin()+index_cir_obs);

      // Don't decrement i because we are looping through matched_tar, not cir_obs
      index_cir_obs--;
    }
    index_cir_obs++;
  }
}

void addNewObs(std::vector<CircleMatch> cm, std::vector<Circle> cirs)
{
  // Determine if we need to modify cir_obs
  // If any new circles could not be matched
  std::vector<int> matched_cirs(cirs.size(), 0);
  for(int i=0;i<cm.size();i++)
  {
    matched_cirs[ cm[i].i_cirs ]++;
  }

  // Create new filters
  for(int i=0;i<cirs.size();i++)
  {
    ////ROS_INFO("matched_cirs[%i]: %i", i, matched_cirs[i]);
    if(!matched_cirs[i])
    {
      ////ROS_INFO("Creating new filter! Center: (%f,%f) Radius: %f", cirs[i].center.x, cirs[i].center.y, cirs[i].radius);
      CircleOb* temp = createCircleOb(cirs[i]);
      ////ROS_INFO("i: %i cir_obs.size(): %i", i, (int)cir_obs.size());
      //cir_obs.insert(cir_obs.begin()+i, temp);
      cir_obs.push_back(temp);
    }
  }
}


/*
 * Compare new circles to old circles and determine matches
 * cir_obs[i]->cir values are set here
 */
std::vector<CircleMatch> dataAssociation(std::vector<Circle> cirs)
{
  ////ROS_INFO("Starting data association");
  ////ROS_INFO("cirs.size(): %i prev_valid_cirs: %i cir_obs: %i", (int)cirs.size(), (int)prev_valid_cirs.size(), (int)cir_obs.size());
  
  std::vector<CircleMatch> cm = matchCircles(cirs, prev_valid_cirs);
  ////ROS_INFO("cm.size(): %i", (int)cm.size());
  ////ROS_INFO("Matching result:");
 
  std::vector<Circle> copy = cirs;
  for(int i=0;i<cm.size();i++)
  {
    ////ROS_INFO("Match %i: i_cirs: %i i_prevCir: %i dist: %f delta_r: %f", i, cm[i].i_cirs, cm[i].i_prevCir, cm[i].dist, cm[i].delta_r);
    
    /*
     * This may need to be looked at later on
     * what if cir_obs.size() == 0? or less than cm.size()? or cm[i].i_prevCir?
     * cir_obs doesn't get new objects pushed on until addNewObs is called
     */
    // Set cir value for this circle obstacle!
    if(cir_obs.size() > cm[i].i_prevCir)
    {
      cir_obs[cm[i].i_prevCir]->cir = copy[cm[i].i_cirs];
      ////ROS_INFO("Setting cir_obs[%i]->cir to (%f,%f)", cm[i].i_prevCir, copy[cm[i].i_cirs].center.x, copy[cm[i].i_cirs].center.y);
    }
  } // end for each potential match
 
  // Delete and add filters based on matching results
  deleteOldObs(cm); 
  addNewObs(cm, cirs);


  /*////ROS_INFO("Done with data association, cir_obs.size(): %i", (int)cir_obs.size());
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("cir_obs[%i] circle: (%f,%f)", i, cir_obs[i]->cir.center.x, cir_obs[i]->cir.center.y);
    if(cir_obs[i]->prevCirs.size()>0)
    {
      ////ROS_INFO("Prev: (%f,%f)", cir_obs[i]->prevCirs[cir_obs[i]->prevCirs.size()-1].center.x, cir_obs[i]->prevCirs[cir_obs[i]->prevCirs.size()-1].center.y);
    }
    else
    {
      ////ROS_INFO("No prev");
    }
  }*/

  return cm;
}


std::vector<Circle> updateKalmanFilters(std::vector<Circle> cirs, std::vector<CircleMatch> cm)
{
  std::vector<Circle> result;
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
    y[0] = cir_obs[i]->cir.center.x;
    y[1] = cir_obs[i]->cir.center.y;
    for(int i=2;i<STATE_SIZE;i++)
    {
      y[i] = 0;
    }
    ////ROS_INFO("Measurement: (%f, %f)", y[0], y[1]);
    ////ROS_INFO("Prediction: (%f, %f)", u[0], u[1]);


    // Update the Kalman filter
    cir_obs[i]->kf->update(u, y);
    ////ROS_INFO("Posterior after update:");
    cir_obs[i]->kf->printPosterior();
    
    // Set new circle center
    MatrixWrapper::ColumnVector mean = cir_obs[i]->kf->posterior->ExpectedValueGet();
    cir_obs[i]->cir.center.x = mean[0];
    cir_obs[i]->cir.center.y = mean[1];

    // Push on resulting circle
    result.push_back(cir_obs[i]->cir);

    // Push back center data for logging
    cirs_pos.push_back(cir_obs[i]->cir);
  }

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


void removeWallObs(std::vector<Circle>& cirs)
{
  int i=0;
  if(ranges.size() > 1)
  {
    while(i<cirs.size())
    {
      Point c = getGlobalCoords(cirs[i]);
      ////ROS_INFO("Circle %i global coordinates: (%f, %f)", i, c.x, c.y);
      
      if( c.x < ranges[0].min || c.x > ranges[0].max ||
          c.y < ranges[1].min || c.y > ranges[1].max )
      {
        ////ROS_INFO("Obstacle is too close to boundary, discarding it");
        cirs.erase(cirs.begin()+i);
        i--;
      }
      else
      {
        ////ROS_INFO("Keeping obstacle");
      }
      i++;
    }
  }
  else
  {
    ////ROS_WARN("Not removing wall obstacles, ranges.size(): %i", (int)ranges.size());
  }
}


void halfCostmap(const nav_msgs::OccupancyGridConstPtr grid, nav_msgs::OccupancyGrid& result)
{
  //ROS_INFO("In halfCostmap");
  //ROS_INFO("Costmap info: width: %i height: %i origin: (%f,%f)", grid->info.width, grid->info.height, grid->info.origin.position.x, grid->info.origin.position.y);
  
  for(int c=grid->info.height/2;c<grid->info.height;c++)
  {
    int c_offset = (c*grid->info.width);// + grid->info.width/2;
    for(int r=grid->info.width/2;r<grid->info.width;r++)
    {
      //////ROS_INFO("c: %i c_offset: %i r: %i total: %i", c, c_offset, r, c_offset+r);
      result.data.push_back(grid->data[c_offset + r]);
    }
  }

  result.info = grid->info;
  result.info.width /= 2;
  result.info.height /= 2;
  result.info.origin.position.x += (result.info.width * result.info.resolution);
  result.info.origin.position.y += (result.info.height * result.info.resolution);
  
  //ROS_INFO("Half Costmap info: width: %i height: %i origin: (%f,%f)", result.info.width, result.info.height, result.info.origin.position.x, result.info.origin.position.y);

  //ROS_INFO("Exiting halfCostmap");
}


void costmapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  ////ROS_INFO("**************************************************");
  ////ROS_INFO("In costmapCb");
  ////ROS_INFO("**************************************************");
  ros::Duration d_elapsed = ros::Time::now() - t_last_costmap;
  t_last_costmap = ros::Time::now();

  /*
   * Only use half of the costmap since the kinect can only see in front of the robot
   */
  //nav_msgs::OccupancyGrid half;
  //halfCostmap(grid, half);

  ////ROS_INFO("New costmap size: %i", (int)grid->data.size());

  double grid_resolution = grid->info.resolution; 
  global_grid = *grid;
  
  //double grid_resolution = half.info.resolution; 
  //global_grid = half;

  ////ROS_INFO("Resolution: width: %i height: %i", grid->info.width, grid->info.height);
  // Consolidate this occupancy grid with prev ones
  nav_msgs::OccupancyGrid consolidated_grid;
  //consolidateCostmaps(half, prev_grids, consolidated_grid);
  consolidateCostmaps(*grid, prev_grids, consolidated_grid);
  
  ////ROS_INFO("Finished getting consolidated_grid");
  
  // Push this grid onto prev_grids
  prev_grids.push_back(global_grid);
  if(prev_grids.size() > num_costmaps_accumulate)
  {
    prev_grids.erase(prev_grids.begin(), prev_grids.begin()+1);
  }

  // Publish the modified costmap(s)
  //pub_half_costmap.publish(half);
  pub_cons_costmap.publish(consolidated_grid);

  // Make a pointer for the modified costmap
  boost::shared_ptr<nav_msgs::OccupancyGrid> cg_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(consolidated_grid);
  //boost::shared_ptr<nav_msgs::OccupancyGrid> cg_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(half);

  ////ROS_INFO("consolidated_grid.data.size(): %i", (int)consolidated_grid.data.size());
  ////ROS_INFO("cg_ptr->data.size(): %i", (int)cg_ptr->data.size());

  /*
   ********************************************
   * Finding circles on latest costmap
   ********************************************
   */

  CirclePacker c(cg_ptr); // (If using modified costmap)
  //CirclePacker c(grid); // (If not modifying costmap)
  //std::vector<Circle> cirs_myblobs = c.go();
  //std::vector<Circle> cirs = c.goHough();
  //std::vector<Circle> cirs = c.goMinEncCir();
  std::vector<Circle> cirs = c.goMyBlobs();

  ////ROS_INFO("Finished getting cirs");

  /*
   * Discard any circles too close to boundaries because those are likely walls
   */
  removeWallObs(cirs);

  ////ROS_INFO("Finished removing wall obstacles");
  

  /*
   * Combine overlapping circles
   */
  // This seg faults if I don't check size > 0
  // Figure out why...
  std::vector<Circle> over;
  if(cirs.size() > 0)
  {
    c.combineOverlappingCircles(cirs, over);
  }

  cirs = over;
  ////ROS_INFO("cirs array finalized:");
  for(int i=0;i<cirs.size();i++)
  {
    ////ROS_INFO("i: %i", i);
    ////ROS_INFO("Circle %i: (%f,%f) radius: %f", i, cirs[i].center.x, cirs[i].center.y, cirs[i].radius);
  }


  /*
   ********************************************
   * Done finding circles on latest costmap
   ********************************************
   */

  /*
   * Convert centers radii to the global frame
   */ 
   double x_origin = global_grid.info.origin.position.x;
   double y_origin = global_grid.info.origin.position.y;
   ////ROS_INFO("About to convert to global coordinates, cirs.size(): %i cir_obs.size(): %i", (int)cirs.size(), (int)cir_obs.size());
   ////ROS_INFO("x_origin: %f y_origin: %f", x_origin, y_origin);
   for(int i=0;i<cirs.size();i++)
   {
     ////ROS_INFO("cir_obs[%i] center: (%f,%f): ", i, cirs[i].center.x, cirs[i].center.y);
     double x = (cirs[i].center.x * global_grid.info.resolution) + x_origin;
     double y = (cirs[i].center.y * global_grid.info.resolution) + y_origin;
     cirs[i].center.x = x;
     cirs[i].center.y = y;
     cirs[i].radius *= global_grid.info.resolution;
     ////ROS_INFO("New Point: (%f,%f) New Radius: %f ", x, y, cirs[i].radius);
   }
   

  /*
   * Data association
   * Do the data associate before kf updates so that 
   * 1) cir_obs.cir is set before doing update
   * 2) the measurement for each cir_ob is correct before doing update
   */
  std::vector<CircleMatch> cm = dataAssociation(cirs);

  ////ROS_INFO("After data association:");
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("cir_obs[%i]->cir.center.x: %f cir_obs[%i]->cir.center.y: %f cir_obs[%i]->cir.radius: %f", i, cir_obs[i]->cir.center.x, i, cir_obs[i]->cir.center.y, i, cir_obs[i]->cir.radius);
  }


  
  /*
   * Call the Kalman filter
   */
   std::vector<Circle> circles_current = updateKalmanFilters(cirs, cm);

  ////ROS_INFO("After kalman filter:");
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("cir_obs[%i]->cir.center.x: %f cir_obs[%i]->cir.center.y: %f cir_obs[%i]->cir.radius: %f", i, cir_obs[i]->cir.center.x, i, cir_obs[i]->cir.center.y, i, cir_obs[i]->cir.radius);
  }

  
   
  /*
   * Circle positions are finalized at this point
   */




   /*
    * Compute orientations
    */
  
  // Check the costmap frequency
  if(num_costmaps % num_costmap_freq_theta == 0 || cirs.size() > prev_valid_cirs.size())
  {
    // Average the theta values
    std::vector<double> thetas = predictTheta();
    for(int i=0;i<cir_obs.size();i++)
    {
      ////ROS_INFO("Obstacle %i", i);
      ////ROS_INFO("theta[%i]: %f", i, thetas[i]);
      cir_obs[i]->prevTheta.push_back(thetas[i]);
      if(cir_obs[i]->prevTheta.size() > num_theta_count)
      {
        ////ROS_INFO("Removing %f", cir_obs[i]->prevTheta[0]);
        cir_obs[i]->prevTheta.erase( cir_obs[i]->prevTheta.begin(), cir_obs[i]->prevTheta.begin()+1 );
      }

      double theta = cir_obs[i]->prevTheta[0];
      for(int j=1;j<cir_obs[i]->prevTheta.size();j++)
      {
        theta = util.findAngleBetweenAngles(theta, cir_obs[i]->prevTheta[j]);
        ////ROS_INFO("j: %i theta: %f next theta: %f result: %f", j, theta, cir_obs[i]->prevTheta[j], util.findAngleBetweenAngles(theta, cir_obs[i]->prevTheta[j]));
      }

      // Set new theta value
      cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] = theta;
    }
  } // end if costmap is considered for predicting theta values



  
  /*
   * Predict velocities
   */
  std::vector<Velocity> velocities = predictVelocities(cm, d_elapsed);

  // Average the velocities
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("i: %i cir_obs.size(): %i velocities.size(): %i cir_obs[%i]->prevTheta.size(): %i", i, (int)cir_obs.size(), (int)velocities.size(), i, (int)cir_obs[i]->prevTheta.size());
    cir_obs[i]->vels.push_back(velocities[i]);
    if(cir_obs[i]->vels.size() > num_velocity_count)
    {
      cir_obs[i]->vels.erase( cir_obs[i]->vels.begin(), cir_obs[i]->vels.begin()+1 );
    }

    float v=0;
    for(int n=0;n<cir_obs[i]->vels.size();n++)
    {
      ////ROS_INFO("n: %i Adding %f", n, cir_obs[i]->vels[n].v);
      v += cir_obs[i]->vels[n].v;
    }
    v /= cir_obs[i]->vels.size();
    ////ROS_INFO("Averaged v: %f", v);

    float theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : initial_theta;
    float vx = v*cos(theta);
    float vy = v*sin(theta);

    // Set values
    if(velocities[i].v < 0.1)
    {
      velocities[i].v   = 0;
      velocities[i].vx  = 0;
      velocities[i].vy  = 0;
    }
    else
    {
      velocities[i].v   = v;
      velocities[i].vx  = vx;
      velocities[i].vy  = vy;
    }
    ////ROS_INFO("Velocity %i: v: %f vx: %f vy: %f w: %f", i, velocities[i].v, velocities[i].vx, velocities[i].vy, velocities[i].w);
    //cir_obs[i]->vel = velocities[i];
  }


  // Push on velocities for data
  prev_velocities.push_back(velocities);
  
  /*
   * Set previous circles
   */
  for(int i=0;i<cir_obs.size();i++)
  {
    cir_obs[i]->prevCirs.push_back(cir_obs[i]->cir);
  }

  // Set prev_cirs variable!
  prev_valid_cirs = circles_current;
 
  /*
   *  After finding velocities, populate Obstacle list
   */  
  obs.clear();
  list.obstacles.clear();

  // Populate list
  for(int i=0;i<cir_obs.size();i++)
  {
    ////ROS_INFO("Creating obstacle, prevCirs.size(): %i prevTheta.size(): %i", (int)cir_obs[i]->prevCirs.size(), (int)cir_obs[i]->prevTheta.size());
    Obstacle o(cir_obs[i]->cir.radius, costmap_width, costmap_height, costmap_origin_x, costmap_origin_y, costmap_res, global_grid.info.origin.position.x, global_grid.info.origin.position.y); 
    
    // Update values
    float theta = cir_obs[i]->prevTheta.size() > 0 ? cir_obs[i]->prevTheta[cir_obs[i]->prevTheta.size()-1] : initial_theta;
    o.update(cir_obs[i]->cir, velocities[i], theta);
  
    obs.push_back(o);
    list.obstacles.push_back(o.msg_);
    ////ROS_INFO("ob %i position: (%f,%f)", i, obs[i].msg_.ob_ms.positions[0], obs[i].msg_.ob_ms.positions[1]);
  }

  ////ROS_INFO("Duration: %f", (ros::Time::now()-t_last_costmap).toSec());
  num_costmaps++;
  ////ROS_INFO("**************************************************");
  ////ROS_INFO("Exiting costmapCb");
  ////ROS_INFO("**************************************************");
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
        ////ROS_INFO("Adding %f", predicted_velocities[i].v);
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
        ////ROS_INFO("Adding %f", predicted_velocities[i].w);
        average_w += predicted_velocities[i].w;
        count_w++;
      }
    }
    average_v /= count_v;
    average_w /= count_w;

    printf("\nPredicted Velocities range=[%f,%f], average: %f\n", min_v, max_v, average_v);
    printf("\nPredicted Velocities range=[%f,%f], average: %f\n", min_w, max_w, average_w);
  }

  ////ROS_INFO("Average differences in circle detection");
  double d=0;
  int count=0;
  for(int i=0;i<d_avg_values.size();i++)
  {
    ////ROS_INFO("d_avg_values[%i]: %f", i, d_avg_values[i]);
    if(!std::isnan(d_avg_values[i]))
    {
      d+=d_avg_values[i];
      count++;
    }
  }
  d = count > 0 ? d/count : 0;
  ////ROS_INFO("Final average difference: %f", d);

  /*for(int i=0;i<cirs_pos.size();i++)
  {
    ////ROS_INFO("cir_pos[%i]: (%f, %f)", i, cirs_pos[i].center.x, cirs_pos[i].center.y);
  }*/


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

  // Add this line to keep the node from hanging
  // Not sure why it hangs without this line
  ros::shutdown();
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

  /*if(handle.hasParam("/ramp/obstacle_odoms"))
  {
    ////ROS_INFO("Found rosparam obstacle_odoms");
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ////ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ////ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
      topic_index_map[ob_odoms.at(i)] = i;
    }
  }
  else
  {
    ////ROS_ERROR("ramp_sensing: Could not find obstacle_topics rosparam!");
  }

  if(handle.hasParam("/ramp/sensing_cycle_rate"))
  {
    handle.getParam("/ramp/sensing_cycle_rate", rate);
    ////ROS_INFO("Sensing cycle rate: %f", rate);
  }
  else
  {
    ////ROS_ERROR("ramp_sensing: Could not find sensing_cycle_rate rosparam!");
  }

  loadObstacleTF();

  // Create subscribers
  std::vector< ros::Subscriber > subs_obs;
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    Obstacle temp;
    temp.T_w_init_ = ob_tfs[i];
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);

    ros::Subscriber sub_ob = handle.subscribe<nav_msgs::Odometry>(ob_odoms.at(i), 1, boost::bind(updateOtherRobotCb, _1, ob_odoms.at(i)));
    subs_obs.push_back(sub_ob);
  } // end for*/

  loadParameters(handle);


  // Initialize the Kalman Filter
  init_linear_system_model();
  init_measurement_model();
  init_prior_model();


  ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  pub_cons_costmap = handle.advertise<nav_msgs::OccupancyGrid>("consolidated_costmap", 1);
  pub_half_costmap = handle.advertise<nav_msgs::OccupancyGrid>("half_costmap", 1);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
  timer_markers = handle.createTimer(ros::Duration(1.f/10.f), publishMarkers);

  
  // Set function to run at shutdown
  signal(SIGINT, reportPredictedVelocity);
   

  printf("\nSpinning\n");

  ros::AsyncSpinner spinner(8);
  printf("\nWaiting for requests...\n");
  spinner.start();
  ros::waitForShutdown();

  printf("\nExiting normally\n");
  return 0;
}
