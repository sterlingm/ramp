#include "evaluate.h"

Evaluate::Evaluate() : Q_coll_(10000.f), Q_kine_(100000.f), orientation_infeasible_(0), T_norm_(50), A_norm_(PI), D_norm_(1.0), T_weight_(1), A_weight_(1), D_weight_(1) {}

void Evaluate::perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res)
{
  ROS_INFO("In Evaluate::perform()");
  //ros::Time t_start = ros::Time::now();
  
  /*
   * Initialize some class memebers
   */
  // Set orientation members
  orientation_.currentTheta_  = req.currentTheta;
  orientation_.theta_at_cc_   = req.theta_cc;

  // Set imminent collision
  imminent_collision_ = req.imminent_collision;
  //////ROS_INFO("imminent_collision_: %s", imminent_collision_ ? "True" : "False");

  // Reset orientation_infeasible for new trajectory
  orientation_infeasible_ = false;




  /*
   * Compute feasibility
   */
  if(req.hmap_eval)
  {
    performFeasibilityHmap(req);
    ROS_INFO("qr.packed_.innerColl_: %s", qrPacked_.innerColl_ ? "True" : "False");
  }
  else
  {
    performFeasibility(req);
  }

  // Set response members
  res.feasible = (!qr_.collision_ && !orientation_infeasible_) && !qrPacked_.innerColl_;
  req.trajectory.feasible = res.feasible;
  ////ROS_INFO("qr_.collision: %s orientation_infeasible_: %s", qr_.collision_ ? "True" : "False", orientation_infeasible_ ? "True" : "False");
  ////ROS_INFO("performFeasibility: %f", (ros::Time::now()-t_start).toSec());

  if(qr_.collision_)
  {
    //////ROS_INFO("Not feasible");
    res.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    //////ROS_INFO("Feasible");
    res.t_firstCollision = ros::Duration(9999.f);
  }






  /*
   * Compute fitness
   */
  // Check what kind of evaluation we are doing
  if(req.hmap_eval)
  {
    double fit;
    performFitnessHmap(req.trajectory, qrPacked_, fit);
    //ROS_INFO("fit: %f", fit);
    
    res.fitness = fit;
    //ROS_INFO("qrPacked_.p_max: %i fit: %f", qrPacked_.p_max_, fit);
  }
  else if(req.full_eval)
  {
    //ROS_INFO("Requesting fitness!");
    performFitness(req.trajectory, req.offset, req.hmap_eval, res.fitness);
  }


  
  ////ROS_INFO("performFitness: %f", (ros::Time::now()-t_start).toSec());
}


void Evaluate::getBoundaryCost(ramp_msgs::RampTrajectory& trj, double& result) const
{
  double d_min = 100;
  // Go through the knot points and check distance to environment bounds
  for(int i=1;i<trj.holonomic_path.points.size()-1;i++)
  {
    std::vector<double> p = trj.holonomic_path.points[i].motionState.positions;
    double dist_x_min = fabs(p[0] - 0);
    double dist_x_max = fabs(p[0] - 3.5);
    double dist_y_min = fabs(p[1] - 0);
    double dist_y_max = fabs(p[1] - 3.5);

    if(dist_x_min < d_min)
    {
      d_min = dist_x_min;
    }
    if(dist_x_max < d_min)
    {
      d_min = dist_x_max;
    }
    if(dist_y_min < d_min)
    {
      d_min = dist_y_min;
    }
    if(dist_y_max < d_min)
    {
      d_min = dist_y_max;
    }
  } // end for
  ROS_INFO("d_min: %f", d_min);

  if(d_min < 0.0001)
  {
    d_min = 0.0001;
  }

  result = d_min;
}


// Modifies the trajectory member of EvaluationRequest&
void Evaluate::performFeasibility(ramp_msgs::EvaluationRequest& er) 
{
  ROS_INFO("In Evaluate::performFeasibility");
  ros::Time t_start = ros::Time::now();

  // Check collision
  ros::Time t_numeric_start = ros::Time::now();
  cd_.performNum(er.trajectory, er.obstacle_trjs, er.robot_radius, er.obstacle_radii, qr_);
  ros::Duration d_numeric   = ros::Time::now() - t_numeric_start;
  t_numeric_.push_back(d_numeric);

  ROS_INFO("result.collision: %s", qr_.collision_ ? "True" : "False");

  // Set feasibility due to collision
  //////ROS_INFO("feasible: %s", er.trajectory.feasible ? "True" : "False");
  er.trajectory.feasible            = !qr_.collision_;
  er.trajectory.t_firstCollision    = ros::Duration(qr_.t_firstCollision_);

  // Set infeasibility due to orientation
  if(!er.imminent_collision && er.consider_trans && !er.trans_possible)
  {
    //ROS_INFO("In final if statement");
    orientation_infeasible_ = true;
  }
  else
  {
    //ROS_INFO("er.imminent_collision: %s er.consider_trans: %s er.trans_possible: %s", er.imminent_collision ? "True" : "False", er.consider_trans ? "True" : "False", er.trans_possible ? "True" : "False");
  }
  
  ROS_INFO("performFeasibility time: %f", (ros::Time::now() - t_start).toSec());
}


void Evaluate::performFeasibilityHmap(ramp_msgs::EvaluationRequest& er)
{
  //ROS_INFO("In Evaluate::performFeasibilityHmap");

  /*
   ****************************************************************************************************
             PackedObstacle does not include inner radii circles for Hilbert map obstacles 
   ****************************************************************************************************
   */
  std::vector<ramp_msgs::PackedObstacle> obs = er.packed_obs;
  cd_.performPackedObs(er.trajectory, obs, er.robot_radius, hmap_, qrPacked_);


  //ROS_INFO("Exiting Evaluate::performFeasibilityHmap");
}



void Evaluate::performFitnessHmap(ramp_msgs::RampTrajectory& trj, const CollisionDetection::QueryResultPacked& qr, double& result)
{
  double cost=0;
  
  // p_max is an int, convert to double
  double perc = qr.p_max_ / 100.0;

  
  // Check feasibility
  if(trj.feasible)
  {
    ROS_INFO("In trj.feasible");
    // Set cost. Scale the collision penalty with the max probability of the traj being in collision
    cost = 1.0 - perc;
    ROS_INFO("cost: %f", (1.0-perc));
  }
  else
  {
    ROS_INFO("In else");
    cost = (perc * Q_coll_);
    ROS_INFO("cost: %f", perc*Q_coll_);
  }

  // Get boundary cost and normalize it
  double bcost;
  getBoundaryCost(trj, bcost);
  bcost /= 4.94975;
  double bcost_cost = 1.0/bcost;
  bcost_cost *= 0.1;
  ROS_INFO("bcost: %f bcost_cost: %f", bcost, bcost_cost);

  // Add boundary cost to the full cost. Do the inverse of boundary cost b/c it's a distance
  cost += bcost_cost; 

  // Set result
  result = cost > 0 ? 1.0 / cost : 1.00;
}



void Evaluate::getEstimatedRemainingTime(ramp_msgs::RampTrajectory& trj, const double& offset, double& result) const
{
  // p = last non-holonomic point on trajectory
  trajectory_msgs::JointTrajectoryPoint p = trj.trajectory.points.at(trj.trajectory.points.size()-1);
  ////////ROS_INFO("p: %s", utility_.toString(p).c_str());

  // Find knot point index on holonomic path where non-holonomic segment ends
  uint16_t i_end=0;
  for(uint16_t i=0;i<trj.holonomic_path.points.size();i++)
  {
    ////////ROS_INFO("i: %i trj.holonomic_path.points.size(): %i", (int)i, (int)trj.holonomic_path.points.size());
    double dist = utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, p.positions);

    //ROS_INFO("trj.holonomic_path[%i]: %s", (int)i, utility_.toString(trj.holonomic_path.points[i].motionState).c_str());
    //ROS_INFO("dist: %f", dist);
    //ROS_INFO("offset: %f", offset);

    // Account for some offset
    if( dist*dist < (0.2 + offset) )
    {
      i_end = i; 
      break;
    }
  } // end for
  
  ////ROS_INFO("i_end: %i", (int)i_end);
  ////ROS_INFO("trj.holonomic_path.points.size(): %i", (int)trj.holonomic_path.points.size());

  /*
   * Estimate time needed for remaining segments
   */
  // For each segment in remaining holonomic path,
  // accumulate the distance and orientation change needed for remaining segment
  double dist=0;
  double delta_theta=0;
  double last_theta = p.positions[2];
  for(uint8_t i=i_end;i<trj.holonomic_path.points.size()-1;i++)
  {
    ////////ROS_INFO("i: %i", (int)i);
    dist += utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
    
    double theta = utility_.findAngleFromAToB(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
    
    delta_theta += fabs(utility_.findDistanceBetweenAngles(last_theta, theta));
    
    last_theta = theta;
  }
  ////ROS_INFO("dist: %f delta_theta: %f", dist, delta_theta);

  // Set velocity values
  double max_v=0.25/2;
  double max_w=PI/8.f;

  // Estimate how long to execute positional and angular displacements based on max velocity
  double estimated_linear   = dist / max_v;
  double estimated_rotation = delta_theta / max_w;

  ////ROS_INFO("estimated_linear: %f estimated_rotation: %f", estimated_linear, estimated_rotation);

  /*
   * Set cost variables
   */
  result = estimated_linear + estimated_rotation;
}



/** This method computes the fitness of the trajectory_ member */
void Evaluate::performFitness(ramp_msgs::RampTrajectory& trj, const double& offset, bool hmap, double& result) 
{
  ////ROS_INFO("In Evaluate::performFitness");
  ros::Time t_start = ros::Time::now();
  
  double cost=0;
  double penalties = 0;

  if(trj.feasible)
  {
    ////ROS_INFO("In if(feasible)");
    
    /*
     * Set cost variables
     */

    // Get total time to execute trajectory
    double T = trj.trajectory.points.at(trj.trajectory.points.size()-1).time_from_start.toSec();

    /*
     * Trajectory point generation ends at the end of the non-holonomic segment
     * For the remaining segments, estimate the time and orientation change needed to execute them
     */
    double estimatedRemaining;
    getEstimatedRemainingTime(trj, offset, estimatedRemaining);
    T += estimatedRemaining;

    // Orientation
    double A = orientation_.perform(trj);

    // Minimum distance to any obstacle
    double D = cd_.min_dist_;

    ROS_INFO("T: %f A: %f D: %f", T, A, D);

    // Update normalization for Time if necessary
    if(T > T_norm_)
    {
      T_norm_ = T;
    }

    // Normalize terms
    T /= T_norm_;
    A /= A_norm_;
    D /= D_norm_;

    //ROS_INFO("Normalized terms T: %f A: %f D: %f", T, A, D);

    // Weight terms
    T *= T_weight_;
    A *= A_weight_;
    D *= D_weight_;

    //ROS_INFO("Weighted terms T: %f A: %f D: %f", T, A, D);

    // Compute overall cost
    cost = T + A + (1.f/D);
  }

  else
  {
    //////ROS_INFO("In else(infeasible)"); 
    
    // Add the Penalty for being infeasible due to collision, at some point i was limiting the time to 10s, but I don't know why
    if(trj.t_firstCollision.toSec() > 0 && trj.t_firstCollision.toSec() < 9998)
    {
      //ROS_INFO("In if t_firstCollision: %f", trj.t_firstCollision.toSec());
      //ROS_INFO("Collision penalty: %f",(Q_coll_ / trj.t_firstCollision.toSec()));
         
      penalties += (Q_coll_ / trj.t_firstCollision.toSec());
    }
    else
    {
      //ROS_INFO("Collision penalty (Full): %f",Q_coll_);
      penalties += Q_coll_;
    }

    // If infeasible due to orientation change
    // If there is imminent collision, do not add this penalty (it's okay to stop and rotate)
    if(orientation_infeasible_ && !imminent_collision_)
    {
      //ROS_INFO("In if orientation_infeasible_: %f", orientation_.getDeltaTheta(trj));
      //ROS_INFO("Orientation penalty: %f", Q_kine_ * (orientation_.getDeltaTheta(trj) / PI));

      double delta_theta = orientation_.getDeltaTheta(trj) < 0.11 ? 0.1 : orientation_.getDeltaTheta(trj);

      penalties += Q_kine_ * (delta_theta / PI);
    }
  }

  //////ROS_INFO("cost: %f penalties: %f", cost, penalties);
  result = (1. / (cost + penalties));

  //////////ROS_INFO("performFitness time: %f", (ros::Time::now() - t_start).toSec());
  ////////ROS_INFO("Exiting Evaluate::performFitness");
} //End performFitness
