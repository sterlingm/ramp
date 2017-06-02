#include "ros/ros.h"
#include "ramp_msgs/Range.h"
#include "utility.h"
#include "modifier.h"

Utility u;

int costmap_width, costmap_height;
float costmap_origin_x, costmap_origin_y, costmap_res;
std::vector<ramp_msgs::Range> ranges;
std::vector<double> dof_min;
std::vector<double> dof_max;




bool handleRequest(ramp_msgs::ModificationRequest::Request& req,
                   ramp_msgs::ModificationRequest::Response& res)
{
  /*std::cout<<"\npath_modification: In handleRequest\n";

  std::cout<<"\nNumber of paths received: "<<req.paths.size();
  std::cout<<"\nPaths received:";
  for(unsigned int i=0;i<req.paths.size();i++) {
    std::cout<<"\n"<<u.toString(req.paths.at(i));
  }
  std::cout<<"\noperator: "<<req.op<<"\n";*/

  Modifier mod(req);

  // Set the ranges
  // do this in a better way eventually...
  mod.in_.utility_.standardRanges_ = ranges; 
  mod.chg_.utility_.standardRanges_ = ranges; 
  mod.move_.utility_.standardRanges_ = ranges; 

  res.mod_paths = mod.perform();
  
  //Insert insert(req.paths.at(0));
  //res.mod_paths.push_back(insert.perform());
  
  //Delete del(req.paths.at(0));
  //res.mod_paths.push_back(del.perform());
  
  //Change change(req.paths.at(0));
  //res.mod_paths.push_back(change.perform());

  //Swap swap(req.paths.at(0));
  //res.mod_paths.push_back(swap.perform());
  
  //Crossover cross(req.paths.at(0), req.paths.at(1));
  //res.mod_paths = cross.perform();
  
  /*std::cout<<"\nPath(s) after modification:";
  for(unsigned int i=0;i<res.mod_paths.size();i++) {
    std::cout<<"\n"<<u.toString(res.mod_paths.at(i));
  }*/

  return true;
}


// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  for(unsigned int i=0;i<dof_min.size();i++) 
  {
    ramp_msgs::Range temp;
    temp.min = dof_min.at(i);
    temp.max = dof_max.at(i);
    /*if(i < 2)
    {
      temp.min += 0.25;
      temp.max -= 0.25;
    }*/
    ranges.push_back(temp); 
  }
} // End initDOF



void loadParameters(const ros::NodeHandle& handle)
{
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
    handle.getParam("costmap_node/costmap/resolution", costmap_res);

    ROS_INFO("Got costmap parameters. w: %i h: %i x: %f y: %f res: %f", costmap_width, costmap_height, costmap_origin_x, costmap_origin_y, costmap_res);
  }*/


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
  }
}





int main(int argc, char** argv) 
{
  ros::init(argc, argv, "path_modification");
  srand(time(0)); 

  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("path_modification", handleRequest); 

  loadParameters(handle);

  Utility u;
  u.standardRanges_ = ranges;
  ROS_INFO("Path modification ranges: ");
  for(int i=0;i<u.standardRanges_.size();i++)
  {
    ROS_INFO("Min: %f Max: %f", u.standardRanges_[i].min, u.standardRanges_[i].max);
  }

  ramp_msgs::Path p1;
  for(unsigned int i=0;i<10;i++) {
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(i);
    kp.motionState.positions.push_back(i);
    kp.motionState.positions.push_back(i+2);

    p1.points.push_back(kp);
  }
  
  ROS_INFO("Path before modification: %s", u.toString(p1).c_str());

  // Test change
  //Change change(p1);
  //change.perform();
  //Swap swap(p1);
  //swap.perform();
  Move m(p1);
  m.utility_ = u;
  m.dir_ = PI/3.;
  m.dist_ = 1.2;
  m.r_ = 0.5;
  m.perform();

  ROS_INFO("Path after modification: %s", u.toString(m.path_).c_str());

/*  ramp_msgs::Path p2;
  for(unsigned int i=5;i>0;i--) {
    ramp_msgs::KnotPoint kp;
    kp.configuration.K.push_back(i);
    kp.configuration.ranges.push_back(r); 

    p2.points.push_back(kp);
  }
  
  std::cout<<"\nPath p2:"<<u.toString(p2);
  //Insert insert(p);
  //Delete del(p);
  //Change cha(p);
  //Swap swap(p);
  Crossover cross(p1, p2);

  //ramp_msgs::Path a = insert.perform();
  //ramp_msgs::Path a = del.perform();
  //ramp_msgs::Path a = cha.perform();
  //ramp_msgs::Path a = swap.perform();
  std::vector<ramp_msgs::Path> as = cross.perform();
  std::cout<<"\nnew path1:"<<u.toString(as.at(0));
  std::cout<<"\nnew path2:"<<u.toString(as.at(1));*/


  std::cout<<"\nSpinning...\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
