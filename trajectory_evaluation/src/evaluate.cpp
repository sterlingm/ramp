#include "evaluate.h"

Evaluate::Evaluate() : Q(1000.f), orientation_infeasible_(0) {}

void Evaluate::perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res)
{
  //ROS_INFO("In Evaluate::perform()");
  //ros::Time t_start = ros::Time::now();
  
  // Set orientation members
  orientation_.currentTheta_  = req.currentTheta;
  orientation_.theta_at_cc_   = req.theta_cc;

  imminent_collision_ = req.imminent_collision;
  ROS_INFO("imminent_collision_: %s", imminent_collision_ ? "True" : "False");

  performFeasibility(req);
  res.feasible = !qr_.collision_;
  //ROS_INFO("performFeasibility: %f", (ros::Time::now()-t_start).toSec());

  if(qr_.collision_)
  {
    //ROS_INFO("Not feasible");
    res.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    //ROS_INFO("Feasible");
    res.t_firstCollision = ros::Duration(9999.f);
  }

  performFitness(req.trajectory, res.fitness);
  //ROS_INFO("performFitness: %f", (ros::Time::now()-t_start).toSec());
}


// Redo this method at some point
// It's modiftying trj AND returning a value
void Evaluate::performFeasibility(ramp_msgs::EvaluationRequest& er) 
{
  //ROS_INFO("In Evaluate::performFeasibility");
  ros::Time t_start = ros::Time::now();

  // Check collision
  ros::Time t_numeric_start = ros::Time::now();
  cd_.performNum(er.trajectory, er.obstacle_trjs, er.coll_dist, qr_);
  ros::Duration d_numeric   = ros::Time::now() - t_numeric_start;
  t_numeric_.push_back(d_numeric);

  //ROS_INFO("result.collision: %s", qr_.collision_ ? "True" : "False");
  /*ros::Time t_analy_start = ros::Time::now();
  cd_.perform(er.trajectory, er.obstacle_trjs, qr_);
  ros::Duration d_analy = ros::Time::now() - t_analy_start;
  t_analy_.push_back(d_analy);*/

  er.trajectory.feasible            = !qr_.collision_;
  er.trajectory.t_firstCollision    = ros::Duration(qr_.t_firstCollision_);

  ramp_msgs::RampTrajectory* trj = &er.trajectory;
  
  bool moving =     (fabs( sqrt(  (trj->trajectory.points[0].velocities[0]*trj->trajectory.points[0].velocities[0]) +
                                  (trj->trajectory.points[0].velocities[1]*trj->trajectory.points[0].velocities[1]))) > 0)
                ||  (fabs(trj->trajectory.points.at(0).velocities.at(2)) > 0) 
                ? true 
                : false;

  bool moving_on_curve = 
      er.trajectory.curves.size()     > 0 && 
    ( er.trajectory.curves.at(0).u_0  > 0.000001 ||
    (utility_.positionDistance(trj->trajectory.points.at(0).positions, 
       er.trajectory.curves.at(0).controlPoints.at(0).positions) < 0.0001) ) 
    ? true
    : false;
  //ROS_INFO("t_moving_on_curve: %f", (ros::Time::now()-t_after).toSec());


  ROS_INFO("moving: %s moving_on_curve: %s", moving ? "True" : "False", moving_on_curve ? "True" : "False");

  // Check orientation for feasibility
  if(moving && fabs(orientation_.getDeltaTheta(er.trajectory)) > 0.25 && !moving_on_curve)
  {
    ROS_INFO("In if");
    
    if(er.trajectory.i_knotPoints.size() > 2 && er.trajectory.curves.size() < 2)
    {
      ROS_INFO("In inner if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), (int)er.trajectory.curves.size());
      er.trajectory.feasible  = false;
      orientation_infeasible_ = true;
    }
    else if(er.trajectory.i_knotPoints.size() == 2 && er.trajectory.curves.size() < 1)
    {
      ROS_INFO("In inner else if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), (int)er.trajectory.curves.size());
      er.trajectory.feasible  = false;
      orientation_infeasible_ = true;
    }
  }
  
  //ROS_INFO("performFeasibility time: %f", (ros::Time::now() - t_start).toSec());
}



/** This method computes the fitness of the trajectory_ member */
void Evaluate::performFitness(ramp_msgs::RampTrajectory& trj, double& result) 
{
  ROS_INFO("In Evaluate::performFitness");
  ros::Time t_start = ros::Time::now();
  
  double cost=0;
  double penalties = 0;

  if(trj.feasible)
  {
    //ROS_INFO("In if(feasible)");
    double T = trj.trajectory.points.at(trj.trajectory.points.size()-1).time_from_start.toSec();
    double A = orientation_.perform(trj);
    //ROS_INFO("T: %f A: %f", T, A);
    cost = T + A;
  }

  else
  {
    //ROS_INFO("In else(infeasible)"); 
    
    // penalties += orientation_.getPenalty();
    
    //ROS_INFO("trj.t_firstColl: %f", trj.t_firstCollision.toSec());

    // Add the Penalty for being infeasible due to collision, at some point i was limiting the time to 10s, but i don't know why
    if(trj.t_firstCollision.toSec() > 0)
    {
      //ROS_INFO("In if t_firstCollision: %f", trj.t_firstCollision.toSec());
      penalties += (Q / trj.t_firstCollision.toSec());
    }
    else
    {
      penalties += Q;
    }

    // If infeasible due to orientation change
    // If there is imminent collision, do not add this penalty (it's okay to stop and rotate)
    if(orientation_infeasible_ && !imminent_collision_)
    {
      ROS_INFO("In if orientation_infeasible_: %f", orientation_.getDeltaTheta(trj));

      penalties += Q * (orientation_.getDeltaTheta(trj) / PI);
    }
  }

  //ROS_INFO("cost: %f penalties: %f", cost, penalties);
  result = (1. / (cost + penalties));

  //ROS_INFO("performFitness time: %f", (ros::Time::now() - t_start).toSec());
} //End performFitness
