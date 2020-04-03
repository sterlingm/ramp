#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "mobile_base.h"
#include "prediction.h"
#include "line.h"
#include "circle.h"
#include "ros/ros.h"
#include "bezier_curve.h"
#include "ramp_msgs/Population.h"
#include <chrono>
#include <signal.h>
#include <fstream>
#include <ros/package.h>
#include <swri_profiler/profiler.h>
using namespace std::chrono;

Utility utility;
std::vector<double> durs;


void fixDuplicates(ramp_msgs::TrajectoryRequest& req)
{
  SWRI_PROFILE("TrajGen-Main-FixDuplicates");
  int i=0;
  while(i<req.path.points.size()-1)
  {
    ramp_msgs::MotionState a = req.path.points.at(i).motionState;
    ramp_msgs::MotionState b = req.path.points.at(i+1).motionState;

    if(utility.positionDistance(a.positions, b.positions) < 0.01)
    {
      /*ROS_WARN("Consecutive duplicate knot points in path:\nPath[%i]:\n%s\nand\nPath[%i]\n%s\nDist: %f Removing knot point at index %i", 
          i+1,
          utility.toString(a).c_str(),
          i+1,
          utility.toString(b).c_str(),
          utility.positionDistance(a.positions, b.positions),
          i);*/
      req.path.points.erase(req.path.points.begin()+i+1);
      i--;
    }

    i++;
  }
}


bool checkGoal(ramp_msgs::TrajectoryRequest req)
{
  SWRI_PROFILE("TrajGen-Main-CheckGoal");
  // Circle predictions only have one knotpoint
  if(req.path.points.size() == 1)
  {
    return false;
  }

  // Go through each knot point
  for(int i=0;i<req.path.points.size()-1;i++)
  {
    //ROS_INFO("dist: %f", utility.positionDistance(req.path.points[i].motionState.positions, req.path.points[i+1].motionState.positions));
    if(utility.positionDistance(req.path.points[i].motionState.positions, req.path.points[i+1].motionState.positions) > 0.1)
    {
      return false;
    }
  }


  return true;
}


bool requestCallback( ramp_msgs::TrajectorySrv::Request& req,
                      ramp_msgs::TrajectorySrv::Response& res) 
{
  SWRI_PROFILE("TrajGen-Main-RequestCallback|Start");

  //ROS_INFO("In trajectory generator requestCallback");
  high_resolution_clock::time_point tStart = high_resolution_clock::now();
  for(uint8_t i=0;i<req.reqs.size();i++)
  {
    ramp_msgs::TrajectoryRequest treq = req.reqs.at(i); 
    ramp_msgs::TrajectoryResponse tres;
    //ROS_INFO("Trajectory Request Received: %s", utility.toString(treq).c_str());

    /*
     * Check for start == goal
     */
    SWRI_PROFILE("TrajGen-Main-RequestCallback|CheckGoal");
    if(checkGoal(treq))
    {
      tres.trajectory.trajectory.points.push_back(utility.getTrajectoryPoint(treq.path.points.at(0).motionState));
      tres.trajectory.i_knotPoints.push_back(0);
      res.resps.push_back(tres);
      continue;
    }

    // Why treq.segments == 1?
    if(treq.type != PREDICTION && treq.type != TRANSITION && (treq.path.points.size() < 3 || treq.segments == 1))
    {
      //ROS_WARN("Changing type to HOLONOMIC");
      treq.type = HOLONOMIC;
      treq.segments++;
    }

    // Build trajectory
    SWRI_PROFILE("TrajGen-Main-RequestCallback|TrajReq");
    if(treq.type != PREDICTION) 
    {
      fixDuplicates(treq);
      
      MobileBase mobileBase;
      if(!mobileBase.trajectoryRequest(treq, tres))
      {
        tres.error = true;
      }

      tres.trajectory.holonomic_path = treq.path;
    }
    else if(treq.path.points.size() > 0) 
    {
      //ROS_INFO("In prediction");
      Prediction prediction;
      prediction.trajectoryRequest(treq, tres);
    }

    if( tres.trajectory.i_knotPoints[0] == tres.trajectory.i_knotPoints[1] )
    {
      ////ROS_WARN("First two knot points are equal!");
    }
    

    if(tres.trajectory.curves.size() > 0)
    {
      std::vector<double> sp = tres.trajectory.curves[0].segmentPoints[1].positions;
      std::vector<double> cp = tres.trajectory.curves[0].controlPoints[1].positions;

      if(utility.positionDistance(sp,cp) > 0.5)
      {
        ROS_INFO("Trajectory Request Received: %s", utility.toString(treq).c_str());
      }
    }

    //ROS_INFO("Response: %s", utility.toString(tres).c_str());
    
    
    res.resps.push_back(tres);
  }

  duration<double> time_span = duration_cast<microseconds>(high_resolution_clock::now()-tStart);
  durs.push_back( time_span.count() );
  ////ROS_INFO("t_end: %f", (t_end-t_start).toSec());
  return true;
}

void writeData()
{
  // General data files
  std::string directory = ros::package::getPath("trajectory_generator");
  
  std::ofstream f_durs;
  f_durs.open(directory+"/durations.txt");

  for(int i=0;i<durs.size();i++)
  {
    f_durs<<"\n"<<durs[i];
  }

  f_durs.close();
}


void shutdown(int sigint)
{
  writeData();
  ros::shutdown();
}


 //Main function
int main(int argc, char** argv) {

  // Initialize the //ROS node 
  ros::init(argc, argv, "reflexxes");
  ros::NodeHandle n;

  // Variable Declaration
  MobileBase mobileBase;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", requestCallback);

  
 // Set function to run at shutdown
 signal(SIGINT, shutdown);
  

  ros::AsyncSpinner spinner(8);
  spinner.start();
  //ROS_INFO("Spinning ...");
  ros::waitForShutdown();

  ros::shutdown();

  return 0; 
}
