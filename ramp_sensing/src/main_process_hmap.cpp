#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<visualization_msgs/MarkerArray.h>
#include "utility.h"
#include "GridMap2D.h"
#include <vector>
#include "circle_packer.h"
#include "ramp_msgs/HilbertMap.h"
#include "ramp_msgs/ObstacleList.h"
#include "obstacle.h"

using namespace std;
using namespace cv;

Mat hmap_mat;
Mat hmap_edges;


ros::Publisher pub_rviz, pub_obs;
visualization_msgs::MarkerArray inner_radii, outer_radii;
ramp_msgs::ObstacleList hmap_obs;



void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( hmap_mat, hmap_edges, Size(3,3) );
  
  //imshow("after blur src", hmap_edges);
  //waitKey(0);


  // Somehow, lowThreshold is being converted to unsigned int before this point
  // its value is 32767 (-1 for unsigned 4-byte int)
  // Set the value back to 0 for edge detection to work
  int lowThreshold = 0;
  int ratio = 3;
  int kernel_size = 3;

  /// Canny detector
  Canny( hmap_edges, hmap_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  imshow( "hmap_edges", hmap_edges);
  waitKey(0);
}



vector<cv::Point> bhContoursCenter(const vector<vector< cv::Point> >& contours,bool centerOfMass,int contourIdx=-1)
{
  ROS_INFO("In bhContoursCenter");
  ROS_INFO("centerOfMass: %s contourIdx: %i", centerOfMass ? "True" : "False", contourIdx);
  vector<cv::Point> result;
  if (contourIdx > -1)
  {
      if (centerOfMass)
      {
          Moments m = moments(contours[contourIdx],true);
          result.push_back( cv::Point(m.m10/m.m00, m.m01/m.m00));
      }
      else 
      {
          Rect rct = boundingRect(contours[contourIdx]);
          result.push_back( cv::Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
      }
  }
  else 
  {
      if (centerOfMass)
      {
        for (int i=0; i < contours.size();i++)
        {
          if(contours[i].size() > 2)
          {
            for(int j=0;j<contours[i].size();j++)
            {
              ROS_INFO("Contour point[%i][%i]: (%i,%i)", i, j, contours[i][j].x, contours[i][j].y);
            }
            Moments m = moments(contours[i],true);
            ROS_INFO("m.m00: %f m.m10: %f m.m01: %f", m.m00, m.m10, m.m01);
            result.push_back( cv::Point(m.m10/m.m00, m.m01/m.m00));
          }
        }
      }
      else 
      {
        for (int i=0; i < contours.size(); i++)
        {
          Rect rct = boundingRect(contours[i]);
          result.push_back(cv::Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
        }
      }
  }

  return result;
}


vector<cv::Point> bhFindLocalMaximum(InputArray _src,int neighbor=2)
{
  // Get image
  Mat src = _src.getMat();

  Mat peak_img = src.clone();

  // Dilate
  dilate(peak_img,peak_img,Mat(),cv::Point(-1,-1),neighbor);
  //imshow("dilate", peak_img);
  peak_img = peak_img - src;
  resize(peak_img, peak_img, Size(peak_img.cols*4, peak_img.rows*4));
  //imshow("dilate-src", peak_img);


  Mat flat_img;
  erode(src,flat_img,Mat(),cv::Point(-1,-1),neighbor);
  //imshow("erode", flat_img);
  flat_img = src - flat_img;
  //imshow("erode-src", flat_img);


  threshold(peak_img,peak_img,0,255,CV_THRESH_BINARY);
  threshold(flat_img,flat_img,0,255,CV_THRESH_BINARY);
  bitwise_not(flat_img,flat_img);

  //imshow("thr_dil", peak_img);
  //imshow("thr_ero", flat_img);

  // Set 255 to any pixels that flat_img has nonzero value
  peak_img.setTo(cv::Scalar::all(255),flat_img);
  printf("\nDone setTo\n");
  // not operator b/c findContours looks for high-valued regions
  bitwise_not(peak_img,peak_img);
  printf("\nDone bitwise_not\n");

  std::vector<std::vector<cv::Point>> contours;
  findContours(peak_img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  
   /*
    *  Draw contours
    */
  std::vector<Vec4i> hierarchy;
  RNG rng(12345);
  Mat drawing = Mat::zeros( peak_img.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }
  // Done drawing contours

  // Get centroids of the contours
  vector<cv::Point> maxima = bhContoursCenter(contours, true);

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  return bhContoursCenter(contours,true);
}


visualization_msgs::Marker getMarker(Circle cir, int id, bool red, bool longTime=true)
{
  visualization_msgs::Marker result;

  result.header.stamp = ros::Time::now();
  result.header.frame_id = "map";
      
  result.ns = "basic_shapes";
  result.id = id;
  
  result.type = visualization_msgs::Marker::SPHERE;
  result.action = visualization_msgs::Marker::ADD;

  // Set x and y
  double x = cir.center.x;
  double y = cir.center.y;

  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = red ? 0.1 : 0;
  result.pose.orientation.x = 0.0;
  result.pose.orientation.y = 0.0;
  result.pose.orientation.z = 0.0;
  result.pose.orientation.w = 1.0;
      
  double radius = cir.radius;
  //ROS_INFO("result info x: %f y: %f radius: %f", x, y, radius);
  
  /*obs[i].cir_.center.x = x;
  obs[i].cir_.center.y = y;
  obs[i].cir_.radius = radius;*/
  
  // scale values are the diameter so use the radius*2
  result.scale.x = radius*2.00f;
  result.scale.y = radius*2.00f;
  result.scale.z = 0.1;
  result.color.r = red ? 1 : 0;
  result.color.g = red ? 0 : 1;
  result.color.b = 0;
  result.color.a = 0.5;
  result.lifetime = longTime ? ros::Duration(120) : ros::Duration(5);
  ROS_INFO("result.lifetime: %f", result.lifetime.toSec());


  return result;
}

visualization_msgs::Marker getMarker(ramp_msgs::Circle cir, int id, bool red=false, bool longTime=false)
{
  Circle c;
  c.center.x = cir.center.x;
  c.center.y = cir.center.y;
  c.radius = cir.radius;
  return getMarker(c, id, red, longTime);
}


void thresholdHilbertMap(Mat hmap, Mat& result, int thresholdValue)
{
  threshold(hmap, result, thresholdValue, 255, CV_THRESH_BINARY);
  //imshow("threshold", result);
  //waitKey(0);
}


void hmapCombined(const ramp_msgs::HilbertMap& hmap)
{
  gridmap_2d::GridMap2D hmap_gmap(hmap.map, false);

  // Create cv::Mat  
  hmap_mat = hmap_gmap.probMap();
  //cv::transpose(hmap_mat, hmap_mat);
  //imshow("hmap", hmap_mat);

  Mat hmap_thresh;
  int thresholdV = 0;

  // Does normal threshold, gives x>0
  thresholdHilbertMap(hmap_mat, hmap_thresh, thresholdV);
  //imshow("threshold1", hmap_thresh);
  //waitKey(0);

  Mat inv_thresh;
  // Min threshold, x<100
  threshold(hmap_mat, inv_thresh, 98, 255, CV_THRESH_BINARY_INV);
  //imshow("threshold2", inv_thresh);
  //waitKey(0);
  //imshow("hmap_thresh", hmap_thresh);
  //cv::waitKey(0);
  
  // Now, AND the two mats together to get the regions where 0<x<100 
  Mat hmap_regions;
  bitwise_and(hmap_thresh, inv_thresh, hmap_regions);

  //imshow("threshold3", hmap_regions);
  //waitKey(0);
  
  
  // Start circle packing 
  CirclePacker cp(hmap_regions, hmap.map);
  std::vector<CircleGroup> blankListForLargeObs;
  std::vector<CircleGroup> ob_cirs = cp.getGroups(blankListForLargeObs, true);
  
  // Get map details
  double x_origin = hmap.map.info.origin.position.x / hmap.map.info.resolution;
  double y_origin = hmap.map.info.origin.position.y / hmap.map.info.resolution;
  double gamma = hmap.gamma;
  double sigma = sqrt( (1.f/2.f*gamma) );
  ROS_INFO("gamma: %f sigma: %f", gamma, sigma);
  ROS_INFO("x_origin: %f y_origin: %f", x_origin, y_origin);

  
  Velocity v_zero;
  double theta = 0;

  /*
   * Create a PackedObstacle instance for each circle
   */
  hmap_obs.obstacles.clear();
  for(int i=0;i<ob_cirs.size();i++)
  {
    // Convert circles to global coordinates
    for(int j=0;j<ob_cirs[i].packedCirs.size();j++)
    {
      // Convert position and radius to global coords and resolution
      ob_cirs[i].packedCirs[j].center.x = (ob_cirs[i].packedCirs[j].center.x * hmap.map.info.resolution) + hmap.map.info.origin.position.x;
      ob_cirs[i].packedCirs[j].center.y = (ob_cirs[i].packedCirs[j].center.y * hmap.map.info.resolution) + hmap.map.info.origin.position.y;
      ob_cirs[i].packedCirs[j].radius *= hmap.map.info.resolution;
    }

    // Create Obstacle object
    Obstacle o(ob_cirs[i]);
    hmap_obs.obstacles.push_back(o.msg_);
  }

  int id=100;
  // For each Obstacle, make markers for each circle
  for(int i=0;i<hmap_obs.obstacles.size();i++)
  {
    // Size of circle vector will change so we need to store old size
    int N = hmap_obs.obstacles[i].cirGroup.packedCirs.size();
    for(int j=0;j<N;j++)
    {
      //ROS_INFO("i: %i j: %i 100+j+i+N: %i 200+N+i+j: %i", i, j, 100+((j*(i+1))+j)+N, 200+N+i+j);
      inner_radii.markers.push_back(getMarker(hmap_obs.obstacles[i].cirGroup.packedCirs[j], ++id, true, true));
      
      // Increase radius for outer circle
      ramp_msgs::Circle inflated = hmap_obs.obstacles[i].cirGroup.packedCirs[j];
      inflated.radius += sigma;
      
      // Push that circle onto PackedOb vector
      hmap_obs.obstacles[i].cirGroup.packedCirs.push_back(inflated);
      
      outer_radii.markers.push_back(getMarker(inflated, ++id, false, true));
    }
  }
  
  
  ROS_INFO("outer_radii.size(): %i inner_radii.size(): %i hmap_obs.size(): %i", (int)outer_radii.markers.size(), (int)inner_radii.markers.size(), (int)hmap_obs.obstacles.size());
  pub_rviz.publish(outer_radii);
  pub_rviz.publish(inner_radii);
  pub_obs.publish(hmap_obs); 
}



void hmapCb(const ramp_msgs::HilbertMap& hmap)
{
  ROS_INFO("\nIn hmapCb\n");

  gridmap_2d::GridMap2D hmap_gmap(hmap.map, false); 
  
  // Create cv::Mat  
  hmap_mat = hmap_gmap.probMap();
  cv::transpose(hmap_mat, hmap_mat);
  //imshow("hmap", hmap_mat);

  Mat hmap_thresh;
  int threshold = 25;

  // Does normal threshold
  thresholdHilbertMap(hmap_mat, hmap_thresh, threshold);
  //imshow("hmap_thresh", hmap_thresh);
  //cv::waitKey(0);

  /*
   * Do circle packing on all hmap obstacles
   */
  CirclePacker cp(hmap_thresh, hmap.map);
  std::vector<CircleGroup> blankListForLargeObs;
  std::vector<CircleGroup> ob_cirs = cp.getGroups(blankListForLargeObs, true);

  ROS_INFO("ob_cirs.size(): %i", (int)ob_cirs.size());
  for(int i=0;i<ob_cirs.size();i++)
  {
    ROS_INFO("ob_cirs[%i].fitCir.radius: %f", i, ob_cirs[i].fitCir.radius);
    ROS_INFO("ob_cirs[%i].packedCir.size(): %i", i, (int)ob_cirs[i].packedCirs.size());
  }
  ROS_INFO("hmap origin: (%f,%f) resolution: %f", hmap.map.info.origin.position.x, hmap.map.info.origin.position.y, hmap.map.info.resolution);
  
  // Get map details
  double x_origin = hmap.map.info.origin.position.x / hmap.map.info.resolution;
  double y_origin = hmap.map.info.origin.position.y / hmap.map.info.resolution;
  double gamma = hmap.gamma;
  double sigma = sqrt( (1.f/2.f*gamma) );
  ROS_INFO("gamma: %f sigma: %f", gamma, sigma);
  ROS_INFO("x_origin: %f y_origin: %f", x_origin, y_origin);

  
  Velocity v_zero;
  double theta = 0;

  /*
   * Create a PackedObstacle instance for each circle
   */
  hmap_obs.obstacles.clear();
  for(int i=0;i<ob_cirs.size();i++)
  {
    // Convert circles to global coordinates
    for(int j=0;j<ob_cirs[i].packedCirs.size();j++)
    {
      // Convert position and radius to global coords and resolution
      ob_cirs[i].packedCirs[j].center.x = (ob_cirs[i].packedCirs[j].center.x * hmap.map.info.resolution) + hmap.map.info.origin.position.x;
      ob_cirs[i].packedCirs[j].center.y = (ob_cirs[i].packedCirs[j].center.y * hmap.map.info.resolution) + hmap.map.info.origin.position.y;
      ob_cirs[i].packedCirs[j].radius *= hmap.map.info.resolution;
    }

    // Create Obstacle object
    Obstacle o(ob_cirs[i]);
    hmap_obs.obstacles.push_back(o.msg_);
  }

  int id=100;
  // For each Obstacle, make markers for each circle
  for(int i=0;i<hmap_obs.obstacles.size();i++)
  {
    // Size of circle vector will change so we need to store old size
    int N = hmap_obs.obstacles[i].cirGroup.packedCirs.size();
    for(int j=0;j<N;j++)
    {
      //ROS_INFO("i: %i j: %i 100+j+i+N: %i 200+N+i+j: %i", i, j, 100+((j*(i+1))+j)+N, 200+N+i+j);
      inner_radii.markers.push_back(getMarker(hmap_obs.obstacles[i].cirGroup.packedCirs[j], ++id, true, true));
      
      // Increase radius for outer circle
      ramp_msgs::Circle inflated = hmap_obs.obstacles[i].cirGroup.packedCirs[j];
      inflated.radius += sigma;
      
      // Push that circle onto PackedOb vector
      hmap_obs.obstacles[i].cirGroup.packedCirs.push_back(inflated);
      
      outer_radii.markers.push_back(getMarker(inflated, ++id, false, true));
    }
  }
  //ROS_INFO("Done creating Obstacle objects");



  /*for(int i=0;i<obs.size();i++)
  {
    ROS_INFO("Before Obstacle %i: Center - (%f,%f) Radius - %f", i, obs[i].center.x, obs[i].center.y, obs[i].radius);
    resize(hmap_thresh, hmap_thresh, Size(hmap_thresh.cols*4, hmap_thresh.rows*4));
    hmap_thresh.at<uchar>(obs[i].center.x, obs[i].center.y) = 128;
    imshow("center", hmap_thresh);
    waitKey(0);

    obs[i].center.x = (obs[i].center.x * hmap.map.info.resolution) + hmap.map.info.origin.position.x;
    obs[i].center.y = (obs[i].center.y * hmap.map.info.resolution) + hmap.map.info.origin.position.y;
    obs[i].radius *= hmap.map.info.resolution;
    ROS_INFO("After Obstacle %i: Center - (%f,%f) Radius - %f", i, obs[i].center.x, obs[i].center.y, obs[i].radius);
    inner_radii.markers.push_back(getMarker(obs[i], i+obs.size()));

    // Make radius bigger and get new circle
    obs[i].radius += sigma;
    outer_radii.markers.push_back(getMarker(obs[i], i));
  }

  for(int i=0;i<inner_radii.markers.size();i++)
  {
    inner_radii.markers[i].color.r = 255;
    inner_radii.markers[i].color.g = 0;
    inner_radii.markers[i].color.b = 0;
    inner_radii.markers[i].color.a = 1;
    inner_radii.markers[i].pose.position.z += 0.1;
  }*/



  ROS_INFO("outer_radii.size(): %i inner_radii.size(): %i hmap_obs.size(): %i", (int)outer_radii.markers.size(), (int)inner_radii.markers.size(), (int)hmap_obs.obstacles.size());
  pub_rviz.publish(outer_radii);
  pub_rviz.publish(inner_radii);
  pub_obs.publish(hmap_obs); 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_hmap");
  ros::NodeHandle handle;

  //ros::Subscriber sub_hmap = handle.subscribe("/hilbert_map", 10, &hmapCb);
  ros::Subscriber sub_hmap = handle.subscribe("/hilbert_map", 10, &hmapCombined);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  pub_obs = handle.advertise<ramp_msgs::ObstacleList>("hmap_obstacles", 1);
  
  printf("\nSpinning\n");
  ros::spin();

  printf("\nExiting normally\n");
}
