#include "evaluate.h"

Evaluate::Evaluate() : Q(1000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(1000.) 
{
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request req) 
{
  trajectory_ = req.trajectory;
  currentTheta_ = req.currentTheta;
  cd_.obstacle_trjs_ = req.obstacle_trjs;
  cd_.trajectory_ = req.trajectory;
  orientation_.trajectory_ = trajectory_;
  orientation_.currentTheta_ = currentTheta_;
  orientation_.theta_at_cc_ = req.theta_cc;
} //End setRequest


const ramp_msgs::EvaluationRequest::Response Evaluate::perform()
{
  res_.feasible = performFeasibility();

  if(!res_.feasible)
  {
    res_.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    res_.t_firstCollision = ros::Duration(9999.f);
  }

  res_.fitness = performFitness(res_.feasible);

  return res_;
}


bool Evaluate::performFeasibility() 
{
  bool result=true;

  // Check collision
  qr_ = cd_.perform();
  trajectory_.feasible = !qr_.collision_;

  bool moving = (fabs( sqrt( pow(trajectory_.trajectory.points.at(0).velocities.at(0), 2) +
                            pow(trajectory_.trajectory.points.at(0).velocities.at(1), 2))) > 0)
                || (fabs(trajectory_.trajectory.points.at(0).velocities.at(2)) > 0) ?
                true : false;

  bool moving_on_curve = 
    trajectory_.curves.size() > 0 && 
    (trajectory_.curves.at(0).u_0 > 0.000001 ||
    (utility_.positionDistance(trajectory_.trajectory.points.at(0).positions, 
       trajectory_.curves.at(0).controlPoints.at(0).positions) < 0.0001) ) ?
    true
    :
    false;


  ROS_INFO("getDeltaTheta: %f", fabs(orientation_.getDeltaTheta()));
  // Check orientation
  if(moving && fabs(orientation_.getDeltaTheta()) > 0.25 && !moving_on_curve)
  {

    /*if(trajectory_.i_knotPoints.size() == 2 && trajectory_.curves.size() < 1)
    {
      trajectory_.feasible = false;
    }*/

    ROS_INFO("In if");
    if(trajectory_.i_knotPoints.size() > 2 && trajectory_.curves.size() < 2)
    {
      ROS_INFO("In inner if, i_knotPoints.size(): %i curves.size(): %i", (int)trajectory_.i_knotPoints.size(), 
          (int)trajectory_.curves.size());
      trajectory_.feasible = false;
    }
    else if(trajectory_.i_knotPoints.size() == 2 && trajectory_.curves.size() < 1)
    {
      ROS_INFO("In inner else if, i_knotPoints.size(): %i curves.size(): %i", (int)trajectory_.i_knotPoints.size(), 
          (int)trajectory_.curves.size());
      trajectory_.feasible = false;
    }
  }

  // If not feasible, set t_firstCollision
  if(!trajectory_.feasible)
  {
    ROS_INFO("traj.t_firstCollision: %f", trajectory_.t_firstCollision.toSec());
    ROS_INFO("qr_.t_firstCollision: %f", qr_.t_firstCollision_);
    trajectory_.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
    ROS_INFO("traj.t_firstCollision: %f", trajectory_.t_firstCollision.toSec());
  }


  result = trajectory_.feasible;
  return result;
}



/** This method computes the fitness of the trajectory_ member */
const double Evaluate::performFitness(bool feasible) 
{
  double result=0;
  double cost=0;
  double penalties = 0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  
  if(feasible)
  {
    double T = trajectory_.trajectory.points.at(trajectory_.trajectory.points.size()-1).time_from_start.toSec();
    double A = 0;//orientation_.perform();
    cost = T + A;
  }

  //double A = 0.;

  //ROS_INFO("T: %f A: %f", T, A);

  else
  {
    //penalties += orientation_.getPenalty();
    
    // Add the Penalty for being infeasible
    if(trajectory_.t_firstCollision.toSec() > 0 && trajectory_.t_firstCollision.toSec() < 10.0f)
    {
      //penalties += (Q / trajectory_.t_firstCollision.toSec());
    }
    else
    {
      //penalties += Q;
    }
    penalties += Q;
  }

  result = (1. / (cost + penalties));

  return result;
} //End performFitness
