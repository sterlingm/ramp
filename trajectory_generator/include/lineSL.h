#ifndef LINESL_H
#define LINESL_H
#include "utility.h"

#define CYCLE_TIME_IN_SECONDS 0.1

class LineSL {
public:
  
  LineSL();
  ~LineSL();

  const std::vector<ramp_msgs::MotionState>   generatePoints();
  void init(const ramp_msgs::MotionState start, 
            const ramp_msgs::MotionState goal,
            const ros::Duration d_init,
            const ros::Duration d_final,
            const double final_speed=-1);

private:

  ReflexxesData reflexxesData_;
  ramp_msgs::MotionState start_, goal_;
  double max_acc_;
  ros::Duration timeFromStart_;
  ros::Duration timeCutoff_;
  Utility utility_;

  const ramp_msgs::MotionState buildMotionState(const ReflexxesData data);

  void initReflexxes();
  void changeReflexxes();

  // Initialize variables just after receiving a service request
  void setReflexxesCurrent();
  void setReflexxesTarget();
  void setReflexxesSelection();
  
  const ramp_msgs::MotionState spinOnce();

  // Returns true if the target has been reached
  const bool finalStateReached();
  
  double final_speed_;
  ros::Duration d_init_, d_final_;
};

#endif
