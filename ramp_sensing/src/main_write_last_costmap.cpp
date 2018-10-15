#include<ros/ros.h>
#include<ros/console.h>
#include<nav_msgs/OccupancyGrid.h>
#include<sensor_msgs/LaserScan.h>
#include<iostream>
#include<fstream>
#include<signal.h>

std::ofstream f_data;
int costmapTimeStep = 0;
int count = 0;
bool firstRowDone = false;

nav_msgs::OccupancyGrid gridPersist;

void costmapCb(const nav_msgs::OccupancyGrid& grid)
{
  //ROS_INFO("In costmapCb");
  
  gridPersist = grid;

  //ROS_INFO("Exiting costmapCb");
}

void writeGrid()
{
  double resolution = gridPersist.info.resolution; 
  double x_origin   = gridPersist.info.origin.position.x;
  double y_origin   = gridPersist.info.origin.position.y;
  ROS_INFO("resolution: %f x_origin: %f y_origin: %f", resolution, x_origin, y_origin);
  ROS_INFO("gridPersist.info.height: %i gridPersist.info.width: %i", gridPersist.info.height, gridPersist.info.width);


  /*for(int c=0;c<gridPersist.info.width;c++)
  {
    for(int r=0;r<gridPersist.info.height;r++)
    {
      int r_offset = gridPersist.info.width*r;
      int value = gridPersist.data[r_offset + c];// > 0 ? 1 : 0;

      // Convert indices to coordinates
      double x = (c*resolution) + x_origin;
      double y = (r*resolution) + y_origin;
      ROS_INFO("r: %i c: %i x: %f y: %f value: %i", r, c, x, y, value);

      // Write the coordinates and values
      // If first line, don't make newline
      if(r == 0 && c == 0)
      {
        f_data<<x<<","<<y<<","<<value;
      }
      else
      {
        f_data<<"\n"<<x<<","<<y<<","<<value;
      }
    }
  }*/
  


  for(int r=0;r<gridPersist.info.height;r++)
  {
    int r_offset = gridPersist.info.width*r;
    for(int c=0;c<gridPersist.info.width;c++)
    {
      int value = gridPersist.data[r_offset + c];// > 0 ? 1 : 0;

      // Convert indices to coordinates
      double x = (c*resolution) + x_origin;
      double y = (r*resolution) + y_origin;
      ROS_INFO("r: %i c: %i x: %f y: %f value: %i", r, c, x, y, value);

      // Write the coordinates and values
      // If first line, don't make newline
      if(r == 0 && c == 0)
      {
        f_data<<x<<","<<y<<","<<value;
      }
      else
      {
        f_data<<"\n"<<x<<","<<y<<","<<value;
      }
    }
  }
}

void shutdown(int sig)
{
  writeGrid();

  // Shutdown
  f_data.close();
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

  f_data.open("/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/static-map.csv", 
      std::ios::out | std::ios::binary);

  ros::spin();
  return 0;
}
