#ifndef KNOT_POINT_H
#define KNOT_POINT_H
#include "ramp_msgs/KnotPoint.h"
#include "motion_state.h"
#include "utility.h"

class KnotPoint {
  public:

    KnotPoint(); 
    KnotPoint(const MotionState mp);
    KnotPoint(const ramp_msgs::KnotPoint kp);
    ~KnotPoint() {}

    // TODO: Make data member msg_
    // Data Members
    MotionState motionState_;
    unsigned int stopTime_;

    // Methods
    const bool equals(const KnotPoint& kp) const;
    const ramp_msgs::KnotPoint buildKnotPointMsg() const;
    const std::string toString() const;
    
  private:
};

#endif
