#include "control_handler.h"

ControlHandler::ControlHandler(const ros::NodeHandle& h) : handle_(h) 
{
  nb_best_traj_switches = 0;

  pub_bestTrajec_ = handle_.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1);
  pub_population_ = handle_.advertise<ramp_msgs::Population>("population", 1);
  
  pub_imminent_collision_ = handle_.advertise<std_msgs::Bool>("imminent_collision", 1);

  // Set number of obstacles
  std::stringstream str;
  for(uint8_t i=0;i<3;i++)
  {
    ROS_INFO("Creating publisher for ob_IC");
    str<<"/obstacle_"<<(int)i<<"/ob_imminent_collision";
    ROS_INFO("Topic: %s", str.str().c_str());
    ros::Publisher p = handle_.advertise<std_msgs::Bool>(str.str(), 1);
    pub_ob_imminent_collision_.push_back(p);

    str.str(std::string());
    str.clear();
  }
}


void ControlHandler::send(ramp_msgs::RampTrajectory bestTrajec) 
{
  nb_best_traj_switches++;
  pub_bestTrajec_.publish(bestTrajec);
}

void ControlHandler::sendPopulation(ramp_msgs::Population population) 
{
  pub_population_.publish(population);
}


void ControlHandler::sendIC(std_msgs::Bool value)
{
  pub_imminent_collision_.publish(value);
}


void ControlHandler::sendObIC(const int i, std_msgs::Bool value)
{
  pub_ob_imminent_collision_.at(i).publish(value);
}
