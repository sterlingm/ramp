#include<ros/ros.h>
#include<ros/console.h>
#include<nav_msgs/OccupancyGrid.h>
#include<sensor_msgs/LaserScan.h>
#include<iostream>
#include<fstream>
#include<signal.h>

std::ofstream f_data;
std::ofstream f_laser_data;
int costmapTimeStep = 0;
int laserTimeStep = 0;
int count = 0;
bool firstRowDone = false;

void costmapCb(const nav_msgs::OccupancyGrid grid)
{
  ROS_INFO("In costmapCb");

  // Go through each cell, convert to global coordinates, and write to file
  double grid_resolution = grid.info.resolution; 
  double x_origin = grid.info.origin.position.x;
  double y_origin = grid.info.origin.position.y;
  //ROS_INFO("grid_resolution: %f x_origin: %f y_origin: %f", grid_resolution, x_origin, y_origin);

  for(int r=0;r<grid.info.height;r++)
  {
    int r_offset = grid.info.width*r;
    for(int c=0;c<grid.info.width;c++)
    {
      int value = grid.data[r_offset + c] > 0 ? 1 : 0;

      double x = (c*grid_resolution) + x_origin;
      double y = (r*grid_resolution) + y_origin;
      //ROS_INFO("r: %i c: %i x: %f y: %f value: %i", r, c, x, y, value);

      // Convert r and c to global x and y values
      if(costmapTimeStep == 0 && r == 0 && c == 0)
      {
        f_data<<costmapTimeStep<<","<<x<<","<<y<<","<<value;
      }
      else
      {
        f_data<<"\n"<<costmapTimeStep<<","<<x<<","<<y<<","<<value;
      }
    }
  }
  costmapTimeStep++;
  
  count++;
  ROS_INFO("Exiting costmapCb");
}

void laserCb(const sensor_msgs::LaserScan scan)
{

  float angle_start = scan.angle_min;
  float delta_theta = scan.angle_increment;
  //ROS_INFO("angle_start: %f delta_theta: %f", angle_start, delta_theta);

  //ROS_INFO("range_min: %f range_max: %f", scan.range_min, scan.range_max);
  

  for(int i=0;i<scan.ranges.size();i++)
  {
    float angle = angle_start + (delta_theta * i);
    float dist = std::isnan(scan.ranges[i]) ? 3.0 : scan.ranges[i];
    float x = dist * cos(angle);
    float y = dist * sin(angle);

    //ROS_INFO("i: %i angle: %f delta_theta*i: %f", i, angle, delta_theta*i);
    //ROS_INFO("scan.ranges[%i]: %f x: %f y: %f", i, scan.ranges[i], x, y);

    int value = scan.ranges[i] < 3.0 ? 1 : 0;

    if(firstRowDone)
    {
      f_laser_data<<"\n"<<laserTimeStep<<","<<x<<","<<y<<","<<value;
    }
    else
    {
      f_laser_data<<laserTimeStep<<","<<x<<","<<y<<","<<value;
      firstRowDone = true;
    }
  }
  
  laserTimeStep++;
}

void shutdown(int sig)
{
  f_data.close();
  f_laser_data.close();
  printf("\nExiting normally\n");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hilbert_data");
  ros::NodeHandle handle;

  // Set reportData to run on shutdown
  signal(SIGINT, shutdown);

  std::string topicName = argc > 1 ? argv[1] : "/costmap_node/costmap/costmap";
  ros::Subscriber sub_occ = handle.subscribe(topicName, 10, costmapCb);
  ros::Subscriber sub_laser = handle.subscribe("/scan", 10, laserCb);

  f_data.open("/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/occ_map_data.csv", 
      std::ios::out | std::ios::app | std::ios::binary);

  f_laser_data.open("/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/laser_data.csv", 
      std::ios::out | std::ios::app | std::ios::binary);

  ROS_INFO("topicName: %s", topicName.c_str());

  ros::spin();
  return 0;
}
