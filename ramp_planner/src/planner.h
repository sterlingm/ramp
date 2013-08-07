#ifndef PLANNER_H
#define PLANNER
#include "ros/ros.h"
#include "path.h"
#include "trajectory_request_handler.h"

class Planner {
  public:
    Planner();
    Planner(const int p);
    Planner(const unsigned int r);
    Planner(const unsigned int r, const int p);
    ~Planner();

    //Data Members
    std::vector<Path> paths_;
    std::vector<ramp_msgs::Trajectory> population_;
    std::vector<Range> ranges_;
    Configuration start_;
    Configuration goal_;
    TrajectoryRequestHandler* h_traj_req_;
    const unsigned int resolutionRate_;
    
    //Methods
    void initialization();
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequestMsg(const int i_path, const std::vector<float> times) const;
    void trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg);
    void init_handlers(const ros::NodeHandle& h);

  private:
    const int populationSize_;
    ramp_msgs::Trajectory traj_;
};

#endif
