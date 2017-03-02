#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
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
#include "blob_detector.h"


#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussian.h"
//using namespace MatrixWrapper;
//using namespace BFL;


Utility util;
double rate;
ros::Publisher pub_obj, pub_rviz, pub_cons_costmap;
std::vector< Obstacle> obs;
ramp_msgs::ObstacleList list;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;
nav_msgs::OccupancyGrid global_grid;

std::vector<tf::Transform> ob_tfs;

std::vector<Circle> prev_cirs;

// prev_velocities[cycle][circle]
std::vector< std::vector<double> > prev_velocities;

size_t prev_size;

ros::Time t_last_costmap;

int count;


std::vector<nav_msgs::OccupancyGrid> prev_grids;

std::vector<Circle> prev_valid_cirs;
std::vector<ros::Time> prev_times;
std::vector<double> jump_thresholds;
double jump_threshold = 0.5;
double jump_threshold_inc = 0.25;


std::vector<double> predicted_velocities;

ros::Timer timer_markers;

/*********************************
 * Variables for BFL
 *********************************/

/*
 * Linear System
 */
BFL::LinearAnalyticConditionalGaussian* sys_pdf=0;
BFL::LinearAnalyticSystemModelGaussianUncertainty* sys_model=0;


/*
 * Measurement model
 */
//MatrixWrapper::Matrix* H=0;
BFL::LinearAnalyticConditionalGaussian* meas_pdf=0;
BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model=0;



/*
 * Prior distribution
 */
BFL::Gaussian* prior=0;


struct NormalDist1D
{
  double mean;
  double variance;
};


// Initial belief
NormalDist1D init_bel;


void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(std::stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(std::stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}




/** Get the other robot's current odometry information and update the obstacle info */
void updateOtherRobotCb(const nav_msgs::Odometry::ConstPtr& o, const std::string& topic) 
{
  //ROS_INFO("In updateOtherRobotCb");
  //ROS_INFO("topic: %s", topic.c_str());
  int index = topic_index_map[topic];
  //ROS_INFO("index: %i", index);
  
  if(obs.size() < index+1)
  {
    //ROS_INFO("In if obs.size() < index");
    Obstacle temp(*o);
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);
  }
  else
  {
    //ROS_INFO("In else");
    obs.at(index).update(*o);
    list.obstacles.at(index) = obs.at(index).msg_;
  }
} //End updateOtherRobotCb




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  //pub_obj.publish(obstacle.buildObstacleMsg());
  pub_obj.publish(list);
} //End sendList


std::vector<visualization_msgs::Marker> convertObsToMarkers()
{
  std::vector<visualization_msgs::Marker> result;
  
  double x_origin = global_grid.info.origin.position.x;
  double y_origin = global_grid.info.origin.position.y;
  
  //ROS_INFO("Before translate: x_origin: %f y_origin: %f", x_origin, y_origin);

  x_origin /= global_grid.info.resolution;
  y_origin /= global_grid.info.resolution;
  
  //ROS_INFO("After translate: x_origin: %f y_origin: %f", x_origin, y_origin);

  for(int i=0;i<obs.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "/map";
    marker.ns = "basic_shapes";
    marker.id = i;
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // This needs to be generalized to the costmap resolution
    double x = (obs[i].cir_.center.x + x_origin) * global_grid.info.resolution;
    double y = (obs[i].cir_.center.y + y_origin) * global_grid.info.resolution;
    
    //ROS_INFO("Before translation: (%f, %f) After translation: (%f, %f)", obs[i].cir_.center.x, obs[i].cir_.center.y, x, y);
    

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double radius = obs[i].cir_.radius * global_grid.info.resolution;
    ROS_INFO("x: %f y: %f radius: %f", x, y, radius);
    
    obs[i].cir_.center.x = x;
    obs[i].cir_.center.y = y;
    obs[i].cir_.radius = radius;
    
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.1);

    result.push_back(marker);
  }

  return result;
}

void publishMarkers(const ros::TimerEvent& e)
{
  
  // Publish a single Marker
  visualization_msgs::MarkerArray result;

  
  std::vector<visualization_msgs::Marker> markers = convertObsToMarkers();

  //ROS_INFO("Publishing %i markers", (int)markers.size());

  result.markers = markers;

  pub_rviz.publish(result);



  //ROS_INFO("markers.size(): %i", (int)markers.size());
  /*for(int i=0;i<result.markers.size();i++)
  {
    if(i==0 && result.markers.size() > 0)
    {
      ROS_INFO("result.markers.size(): %i", (int)result.markers.size());
      for(int j=0;j<result.markers.size();j++)
      {
        ROS_INFO("Circle %i scale: %f, %f", j, result.markers[j].scale.x, result.markers[j].scale.y);
      }
    }
  }*/
}


int getClosestPrev(Circle m, std::vector<Circle> N)
{
  int min_index = 0;

  std::vector<double> center;
  center.push_back(m.center.x);
  center.push_back(m.center.y);

  //ROS_INFO("center: [%f, %f]", center[0], center[1]);

  if(prev_cirs.size() > 0)
  {

    std::vector<double> prev_center;
    prev_center.push_back(N.at(0).center.x);
    prev_center.push_back(N.at(0).center.y);

    double dist = util.positionDistance(center, prev_center);
    
    //ROS_INFO("Prev center: [%f, %f]", prev_center[0], prev_center[1]);
    //ROS_INFO("dist: %f", dist);

    for(int i=1;i<N.size();i++)
    {
      prev_center.at(0) =  N[i].center.x;
      prev_center.at(1) =  N[i].center.y;

      if( util.positionDistance(center, prev_center) < dist )
      {
        dist =  util.positionDistance(center, prev_center);
        min_index = i;
      }
    }

    //ROS_INFO("min_index: %i dist: %f", min_index, dist);
  }

  else
  {
    return -1;
  }

  return min_index;
}


bool jump(const Circle cir_i, const Circle cir_prev, double threshold)
{
  ROS_INFO("In jump");
  double dist = util.positionDistance(cir_i.center.x, cir_i.center.y, cir_prev.center.x, cir_prev.center.y);

  ROS_INFO("cir_i center: (%f, %f), cir_prev center: (%f, %f) threshold: %f dist: %f", cir_i.center.x, cir_i.center.y, cir_prev.center.x, cir_prev.center.y, threshold, dist);


  return dist > threshold;
}

std::vector<double> velocityFromOnePrev(const std::vector<Circle> cirs, const std::vector<Circle> prev_cirs, const ros::Duration d_elapsed, const double grid_resolution)
{
  std::vector<double> result;

  std::vector<int> cir_prev_cen_index;
  std::vector<double> linear_vs;
  std::vector<double> angular_vs;
  if(cirs.size() == prev_cirs.size() || cirs.size() < prev_cirs.size())
  {
    // Find closest previous circle for each new circle
    for(int i=0;i<cirs.size();i++)
    {
      //int index = getClosestPrev(cirs[i], prev_cirs);
      int index = getClosestPrev(cirs[i], prev_valid_cirs);
      ROS_INFO("Circle %i center: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
      ROS_INFO("Closest prev: Circle %i center: (%f, %f)", i, prev_valid_cirs[index].center.x, prev_valid_cirs[index].center.y);
      cir_prev_cen_index.push_back(index);

      if(jump(cirs[i], prev_valid_cirs[index], jump_thresholds[i]))
      {
        ROS_INFO("Huge jump between previous costmap and this costmap");
        jump_thresholds[i] += jump_threshold_inc;
      }
      else
      {
        ros::Duration d_prev = ros::Time::now() - prev_times[i];
        ROS_INFO("d_prev: %f", d_prev.toSec());

        jump_thresholds[i] = jump_threshold;

        // Find velocities
        std::vector<double> pc;
        pc.push_back(prev_valid_cirs[index].center.x);
        pc.push_back(prev_valid_cirs[index].center.y);
        std::vector<double> cc;
        cc.push_back(cirs[i].center.x);
        cc.push_back(cirs[i].center.y);
        double theta = util.findAngleFromAToB(pc, cc);
        double linear_v = (util.positionDistance(pc, cc) / d_prev.toSec()) * grid_resolution;
        result.push_back(linear_v);
        ROS_INFO("dist: %f time: %f linear_v: %f converted: %f", util.positionDistance(pc, cc), d_prev.toSec(), util.positionDistance(pc, cc) / d_elapsed.toSec(), linear_v);

        prev_times[i] = ros::Time::now();

        predicted_velocities.push_back(linear_v);

        // Set new valid previous circle
        if(i > prev_valid_cirs.size())
        {
          prev_valid_cirs.push_back(cirs[i]);
        }
        else
        {
          prev_valid_cirs[i] = cirs[i];
        }
      } // end else
    } // end for each circle
  } // end outer if

  // This doesn't actually compute velocity, it just sets a new prev_cir
  else
  {
    ROS_INFO("********************************************************************************");
    ROS_INFO("More new circles than previous circles!");
    // Find closest new circle for each prev center
    // Do this because the other way could match 2 new circles to the same prev center
    for(int i=0;i<prev_cirs.size();i++)
    {
      int index = getClosestPrev(prev_cirs[i], cirs);
      ROS_INFO("Circle %i center: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
      ROS_INFO("Closest prev: Circle %i center: (%f, %f)", i, prev_cirs[i].center.x, prev_cirs[i].center.y);
      cir_prev_cen_index.push_back(index);
    }
    ROS_INFO("********************************************************************************");
  }

  return result;
}


// Get N random samples?
std::vector<double> getSamples(const std::vector<double> vels, const int N)
{
  std::vector<double> result;

  return result;
}


// Fit normal distribution to velocity?
// index 0 = mean, 1 = variance
NormalDist1D fitNormal(const std::vector<double> points)
{
  ROS_INFO("In fitNormal");
  ROS_INFO("points.size(): %i", (int)points.size());

  NormalDist1D result;

  double mean = points[0];
  for(int i=0;i<points.size();i++)
  {
    ROS_INFO("Points[%i]: %f", i, points[i]);
    mean += points[i];
  }
  mean /= points.size();
  ROS_INFO("Mean: %f", mean);

  double std=0;
  for(int i=0;i<points.size();i++)
  {
    double diff = pow(points[i] - mean,2);
    std += diff;
  }
  std /= points.size();

  ROS_INFO("Variance: %f", std);

  result.mean = mean;
  result.variance = std;

  ROS_INFO("Exiting fitNormal");
  return result;
}



NormalDist1D addNorm(const NormalDist1D n1, const NormalDist1D n2)
{
  NormalDist1D result;

  result.mean = n1.mean + n2.mean;
  result.variance = n1.variance + n2.variance;

  return result;
}


NormalDist1D multiplyNorm(const NormalDist1D n1, const NormalDist1D n2)
{
  NormalDist1D result;

  double var_result = 1.f / ((1.f/n1.mean) + (1.f/n2.mean));

  double mean_result = ( (n1.variance / n1.mean) + (n2.variance / n2.mean) ) * var_result;

  result.mean = mean_result;
  result.variance = var_result;

  return result;
}




std::vector<double> velocityFromNPrev(const std::vector<Circle> cirs, const std::vector<Circle> prev_cirs, const ros::Duration d_elapsed, const double grid_resolution, int N)
{
  ROS_INFO("In velocityFromNPrev");
  std::vector<double> result;

  // Get velocity from one cycle prev
  result = velocityFromOnePrev(cirs, prev_cirs, d_elapsed, grid_resolution);
  ROS_INFO("velocityFromOnePrev: %f", result.size() > 0 ? result[0] : 0);

  // Build points vector for normal distribution
  std::vector<double> points; 

  // Average the results
  for(int i=0;i<result.size();i++)
  {
    ROS_INFO("Circle %i prev velocities", i);
    for(int j=prev_velocities.size()-2;j>prev_velocities.size()-N-1 && j>-1;j--)
    {
      if(prev_velocities[j].size() > i)
      {
        ROS_INFO("Velocity at prev cycle %i: %f", (int)prev_velocities.size()-j, prev_velocities[j][i]);
        result[i] += prev_velocities[j][i];
        points.push_back(prev_velocities[j][i]);
      }
    }

    if(points.size() > 0)
    {
      NormalDist1D norm = fitNormal(points);
    }

    result[i] /= N;
    ROS_INFO("Average: %f", result[i]);
  }
  

  return result;
}

void prediction(BFL::ExtendedKalmanFilter& filter, const BFL::SystemModel<MatrixWrapper::ColumnVector>* sys_model, const MatrixWrapper::ColumnVector& u)
{
  //filter.SysUpdate(sys_model, u);
}


BFL::ExtendedKalmanFilter makeFilter(BFL::Gaussian prior)
{
  BFL::ExtendedKalmanFilter filter(&prior);
  return filter;
}

void init_measurement_model(Circle temp)
{
  //  z_k+1 = H_x_k+1
  MatrixWrapper::Matrix H(1,2);

  // Set x and y
  H(1,1) = temp.center.x;
  H(1,2) = temp.center.y;

  BFL::ColumnVector meas_noise_mu(1);
  meas_noise_mu(1) = 0; //MU_MEAS_NOISE

  MatrixWrapper::SymmetricMatrix meas_noise_cov(1);
  meas_noise_cov(1,1) = 0;//SIGMA_MEAS_NOISE

  // Make the Gaussian
  BFL::Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);

  // Make the pdf
  meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty);

  // Make model
  meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);
 }

void init_prior_model()
{
  // Build prior distribution
  BFL::ColumnVector prior_mu(2);
  prior_mu(1) = 0; //PRIOR_MU_X
  prior_mu(2) = 0; //PRIOR_MU_Y

  MatrixWrapper::SymmetricMatrix prior_cov(2);
  prior_cov(1,1) = 0; //PRIOR_COV_X
  prior_cov(1,2) = 0.0;
  prior_cov(2,1) = 0.0;
  prior_cov(2,2) = 0; //PRIOR_COV_Y

  prior = new BFL::Gaussian(prior_mu, prior_cov);
}

void init_linear_system_model()
{

  // Build system for x_k+1 = A_x_k + B_u_k
  // x_k is current state...u_k is the control to move to the next state
  // This measures how accurately the robot can follow controls
  MatrixWrapper::Matrix A(2,2);
  A(1,1) = 1.0;
  A(1,2) = 0.0;
  A(2,1) = 0.0;
  A(2,2) = 1.0;

  MatrixWrapper::Matrix B(2,2);
  B(1,1) = cos(0.8);
  B(1,2) = 0.0;
  B(2,1) = 0.0;
  B(2,2) = cos(0.8);

  std::vector<MatrixWrapper::Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // Create Gaussian
  BFL::ColumnVector sys_noise_mu(2);
  sys_noise_mu(1) = 0; // MU_SYSTEM_NOISE_X
  sys_noise_mu(2) = 0; // MU_SYSTEM_NOISE_Y

  // Create covariance martix
  MatrixWrapper::SymmetricMatrix sys_noise_cov(2);
  sys_noise_cov = 0.0;
  sys_noise_cov(1,1) = 0; // SIGMA_SYSTEM_NOISE_X
  sys_noise_cov(1,2) = 0.0;
  sys_noise_cov(2,1) = 0.0;
  sys_noise_cov(2,2) = 0; // SIGMA_SYSTEM_NOISE_Y

  BFL::Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);
  
  // Create pdf
  sys_pdf = new BFL::LinearAnalyticConditionalGaussian(AB, system_uncertainty);

  // Create system model from pdf
  sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(sys_pdf);
}


void consolidateCostmaps(const nav_msgs::OccupancyGrid g1, const nav_msgs::OccupancyGrid g2, nav_msgs::OccupancyGrid& result)
{
  ROS_INFO("In consolidateCostmaps(OccupancyGrid, OccupancyGrid, OccupancyGrid)");
  ROS_INFO("g1.data.size(): %i", (int)g1.data.size());
  result = g1;
  ROS_INFO("Before for loops, result.size(): %i", (int)result.data.size());
  for(int r=0;r<g1.info.width;r++)
  {
    int r_offset = g1.info.width*r;
    for(int c=0;c<g1.info.height;c++)
    {
      result.data[r_offset + c] = g1.data[r_offset + c] | g2.data[r_offset + c];
    }
  }
  ROS_INFO("After for loops, result.size(): %i", (int)result.data.size());
}

void consolidateCostmaps(const nav_msgs::OccupancyGrid gi, const std::vector<nav_msgs::OccupancyGrid> prev_grids, nav_msgs::OccupancyGrid& result)
{
  ROS_INFO("In consolidateCostmaps(OccupancyGrid, vector<OccupancyGrid>, OccupancyGrid)");
  ROS_INFO("gi.size(): %i", (int)gi.data.size());
  
  if(prev_grids.size() == 0)
  {
    result = gi;
  }
  else
  {
    nav_msgs::OccupancyGrid temp = gi;

    for(int i=0;i<prev_grids.size();i++)
    {
      //ROS_INFO("Consolidating with previous grid %i, prev_grid[%i].size(): %i", i, i, (int)prev_grids[i].data.size());
      consolidateCostmaps(temp, prev_grids[i], result);
      //ROS_INFO("New result size: %i", (int)result.data.size());
      temp = result;
    }
  }
}

void costmapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  ROS_INFO("**************************************************");
  ROS_INFO("Got a new costmap!");
  ROS_INFO("**************************************************");
  ros::Duration d_elapsed = ros::Time::now() - t_last_costmap;
  t_last_costmap = ros::Time::now();

  ROS_INFO("New costmap size: %i", (int)grid->data.size());


  // Consolidate this occupancy grid with prev ones
  nav_msgs::OccupancyGrid consolidated_grid;
  consolidateCostmaps(*grid, prev_grids, consolidated_grid);
  
  // Push this grid onto prev_grids
  prev_grids.push_back(*grid);
  if(prev_grids.size() > 5)
  {
    prev_grids.pop_back();
  }

  // Publish the consolidated costmap
  pub_cons_costmap.publish(consolidated_grid);


  //nav_msgs::OccupancyGridPtr cg(&consolidated_grid);
  boost::shared_ptr<nav_msgs::OccupancyGrid> cg_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(consolidated_grid);
  //nav_msgs::OccupancyGridPtr p = cg_ptr;

  //*cg = consolidated_grid;
  ROS_INFO("consolidated_grid.data.size(): %i", (int)consolidated_grid.data.size());
  ROS_INFO("cg_ptr->data.size(): %i", (int)cg_ptr->data.size());


  double grid_resolution = grid->info.resolution; 

  global_grid = *grid;

  CirclePacker c(cg_ptr);
  //CirclePacker c(grid);
  std::vector<Circle> cirs = c.go();

  if(prev_valid_cirs.size() == 0)
  {
    prev_valid_cirs = cirs;
   
    // Make another for 2
    Circle temp;
    prev_valid_cirs.push_back(temp);
  }

  /*
   * Use custom BlobDetector
   */
  /*BlobDetector bd(grid);
  std::vector<Center> cs;
  bd.findBlobs(cs);*/

  
  // This seg faults if I don't check size > 0
  // Figure out why...
  std::vector<Circle> over;
  if(cirs.size() > 0)
  {
    c.combineOverlappingCircles(cirs, over);
    ROS_INFO("over.size(): %i", (int)over.size());
    for(int i=0;i<over.size();i++)
    {
      ROS_INFO("Overlapping Circle %i - Center: (%f, %f) Radius: %f", i, over[i].center.x, over[i].center.y, over[i].radius);
    }
  }

  cirs = over;

  //ROS_INFO("cirs.size(): %i obs.size(): %i", (int)cirs.size(), (int)obs.size());
  /*std::vector<double> velocities = velocityFromOnePrev(cirs, prev_cirs, d_elapsed, grid_resolution);
  ROS_INFO("Velocities returned from going back one cycle:");*/

  int N = 0;
  std::vector<double> velocities = velocityFromNPrev(cirs, prev_cirs, d_elapsed, grid_resolution, N);
  
  for(int i=0;i<velocities.size();i++)
  {
    ROS_INFO("Velocity %i: %f", i, velocities[i]);
  }

  prev_velocities.push_back(velocities);

  
  // Set prev_cirs variable!
  prev_cirs = cirs;

  /*
   *  Compute Circle velocity
   */
  // Create Obstacle for each circle?

  // After finding velocities, populate Obstacle list
  obs.clear();

  for(int i=0;i<cirs.size();i++)
  {
    Obstacle o; 
    o.cir_ = cirs[i];
    obs.push_back(o);
  }

 
  //ROS_INFO("obs.size(): %i", (int)obs.size());
  //ROS_INFO("Leaving Cb");
}


void reportPredictedVelocity(int sig)
{
  timer_markers.stop();


  printf("\npredicted_velocities.size(): %i", (int)predicted_velocities.size());
  double min=predicted_velocities.at(0), max = min, average=min;
  for(int i=1;i<predicted_velocities.size();i++)
  {
    if(predicted_velocities[i] < min)
    {
      min = predicted_velocities[i];
    }
    if(predicted_velocities[i] > max)
    {
      max = predicted_velocities[i];
    }

    if (predicted_velocities[i] > 0)
    {
      ROS_INFO("Adding %f", predicted_velocities[i]);
      average += predicted_velocities[i];
    }
  }
  average /= predicted_velocities.size();

  printf("\nPredicted Velocities range=[%f,%f], average: %f\n", min, max, average);
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
    ROS_INFO("Found rosparam obstacle_odoms");
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
      topic_index_map[ob_odoms.at(i)] = i;
    }
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find obstacle_topics rosparam!");
  }

  if(handle.hasParam("/ramp/sensing_cycle_rate"))
  {
    handle.getParam("/ramp/sensing_cycle_rate", rate);
    ROS_INFO("Sensing cycle rate: %f", rate);
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find sensing_cycle_rate rosparam!");
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

  // Initialize this in a better way later on...
  for(int i=0;i<3;i++)
  {
    prev_times.push_back(ros::Time::now());
    jump_thresholds.push_back(0.5);
  }

  ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  pub_cons_costmap = handle.advertise<nav_msgs::OccupancyGrid>("consolidated_costmap", 1);

  //Timers
  //ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
  timer_markers = handle.createTimer(ros::Duration(1.f/10.f), publishMarkers);

  init_bel.mean = 0;
  init_bel.variance = 5;

  signal(SIGINT, reportPredictedVelocity);
  
   

  std::cout<<"\nSpinning\n";

  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();

  std::cout<<"\nExiting Normally\n";
  return 0;
}
