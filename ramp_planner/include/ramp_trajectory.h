#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/RampTrajectory.h"
#include "path.h"
#include "utility.h"

class RampTrajectory 
{
  public:
    
    explicit RampTrajectory(unsigned int id=0);
    RampTrajectory(const ramp_msgs::RampTrajectory msg);
    ~RampTrajectory() {}
    
    ramp_msgs::RampTrajectory msg_;
    ramp_msgs::MotionState    ms_prevSP_;
    ramp_msgs::RampTrajectory transitionTraj_;

    const RampTrajectory clone()                                const;
    const bool           equals(const RampTrajectory& other)    const;
    const bool           isSame(const RampTrajectory& other)    const;
    const double         getIndexOfMs(const MotionState ms)     const;
    const double         getT()                                 const;
    const Path           getNonHolonomicPath()                  const;
    const double         getDirection()                         const;
    const std::string    fitnessFeasibleToString()              const;
    const std::string    toString(bool printKnotPoints=true)    const;
    void print();
    const RampTrajectory getSubTrajectory(const float t)        const;
    const RampTrajectory getSubTrajectoryPost(const double t)   const;
    
    bool stopRotateInFirstSec() const;

    const RampTrajectory concatenate(const RampTrajectory traj, const uint8_t kp=0) const;
    void concatenateForce(const RampTrajectory traj);

    const trajectory_msgs::JointTrajectoryPoint getPointAtTime(const float t) const;

    void offsetPositions(const MotionState& diff);
    const double reward() const;

  private:
    Utility utility_;
};

#endif
