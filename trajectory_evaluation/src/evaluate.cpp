#include "evaluate.h"

Evaluate::Evaluate() : orientation_infeasible_(0), _1_T_norm_(0.0), _1_A_norm_(0.0), D_norm_(0.0), coll_time_norm_(0.00001),
                       last_T_weight_(-1.0), last_A_weight_(-1.0), last_D_weight_(-1.0), last_Q_coll_(-1.0), last_Q_kine_(-1.0) {}

void Evaluate::perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res)
{
  //ROS_INFO("In Evaluate::perform()");
  //ros::Time t_start = ros::Time::now();
  
  /*
   * Initialize some class memebers
   */
  // Set orientation members
  orientation_.currentTheta_  = req.currentTheta;
  orientation_.theta_at_cc_   = req.theta_cc;

  // Set imminent collision
  imminent_collision_ = req.imminent_collision;
  //ROS_INFO("imminent_collision_: %s", imminent_collision_ ? "True" : "False");

  // Reset orientation_infeasible for new trajectory
  orientation_infeasible_ = false;

  /*
   * Compute feasibility
   */
  performFeasibility(req);

  // Set response members
  req.trajectory.feasible = !qr_.collision_ && !orientation_infeasible_;
  res.feasible = !qr_.collision_ && !orientation_infeasible_;
  req.trajectory.feasible = res.feasible;
  //ROS_INFO("qr_.collision: %s orientation_infeasible_: %s", qr_.collision_ ? "True" : "False", orientation_infeasible_ ? "True" : "False");
  //////ROS_INFO("performFeasibility: %f", (ros::Time::now()-t_start).toSec());

  if(qr_.collision_)
  {
    ////////ROS_INFO("Not feasible");
    res.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    ////////ROS_INFO("Feasible");
    res.t_firstCollision = ros::Duration(9999.f);
  }


  /*
   * Compute fitness
   */
  if(req.full_eval)
  {
    ////ROS_INFO("Requesting fitness!");
    performFitness(req.trajectory, req.offset, res.fitness);
  }
  //////ROS_INFO("performFitness: %f", (ros::Time::now()-t_start).toSec());
}


// Redo this method at some point
// It's modiftying trj AND returning a value
void Evaluate::performFeasibility(ramp_msgs::EvaluationRequest& er) 
{
  ////ROS_INFO("In Evaluate::performFeasibility");
  ros::Time t_start = ros::Time::now();

  // Check collision
  ros::Time t_numeric_start = ros::Time::now();
  cd_.performNum(er.trajectory, er.obstacle_trjs, er.robot_radius, er.obstacle_radii, qr_);
  ros::Duration d_numeric   = ros::Time::now() - t_numeric_start;
  t_numeric_.push_back(d_numeric);

  ////ROS_INFO("result.collision: %s", qr_.collision_ ? "True" : "False");
  /*ros::Time t_analy_start = ros::Time::now();
  cd_.perform(er.trajectory, er.obstacle_trjs, qr_);
  ros::Duration d_analy = ros::Time::now() - t_analy_start;
  t_analy_.push_back(d_analy);*/

  ////////ROS_INFO("feasible: %s", er.trajectory.feasible ? "True" : "False");
  er.trajectory.feasible            = !qr_.collision_;
  er.trajectory.t_firstCollision    = ros::Duration(qr_.t_firstCollision_);

  ramp_msgs::RampTrajectory* trj = &er.trajectory;
  
  if((!er.imminent_collision && er.consider_trans && !er.trans_possible) ||
      orientation_.getDeltaTheta(*trj) > 1.5708) // 90 degree
  {
    ////ROS_INFO("In final if statement");
    orientation_infeasible_ = true;
  }
  else
  {
    ////ROS_INFO("er.imminent_collision: %s er.consider_trans: %s er.trans_possible: %s", er.imminent_collision ? "True" : "False", er.consider_trans ? "True" : "False", er.trans_possible ? "True" : "False");
  }
  
  ////////////ROS_INFO("performFeasibility time: %f", (ros::Time::now() - t_start).toSec());
}



/** This method computes the fitness of the trajectory_ member */
void Evaluate::performFitness(ramp_msgs::RampTrajectory& trj, const double& offset, double& result) 
{
  //////ROS_INFO("In Evaluate::performFitness");
  ros::Time t_start = ros::Time::now();
  
  double cost=0;
  double penalties = 0;

  if(trj.feasible)
  {
    //ROS_INFO("In if(feasible)");
    
    // Get total time to execute trajectory
    double T = trj.trajectory.points.at(trj.trajectory.points.size()-1).time_from_start.toSec();

    /*
     * Trajectory point generation ends at the end of the non-holonomic segment
     * For the remaining segments, estimate the time and orientation change needed to execute them
     */

    // p = last non-holonomic point on trajectory
    trajectory_msgs::JointTrajectoryPoint p = trj.trajectory.points.at(trj.trajectory.points.size()-1);
    //////////ROS_INFO("p: %s", utility_.toString(p).c_str());

    // Find knot point index on holonomic path where non-holonomic segment ends
    uint16_t i_end=0;
    for(uint16_t i=0;i<trj.holonomic_path.points.size();i++)
    {
      //////////ROS_INFO("i: %i trj.holonomic_path.points.size(): %i", (int)i, (int)trj.holonomic_path.points.size());
      double dist = utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, p.positions);

      ////ROS_INFO("trj.holonomic_path[%i]: %s", (int)i, utility_.toString(trj.holonomic_path.points[i].motionState).c_str());
      ////ROS_INFO("dist: %f", dist);
      ////ROS_INFO("offset: %f", offset);

      // Account for some offset
      if( dist*dist < (0.2 + offset) )
      {
        i_end = i; 
        break;
      }
    } // end for
    
    //////ROS_INFO("i_end: %i", (int)i_end);
    //////ROS_INFO("trj.holonomic_path.points.size(): %i", (int)trj.holonomic_path.points.size());

    // For each segment in remaining holonomic path,
    // accumulate the distance and orientation change needed for remaining segment
    double dist=0;
    double delta_theta=0;
    double last_theta = p.positions[2];
    for(uint8_t i=i_end;i<trj.holonomic_path.points.size()-1;i++)
    {
      //////////ROS_INFO("i: %i", (int)i);
      dist += utility_.positionDistance(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
      
      double theta = utility_.findAngleFromAToB(trj.holonomic_path.points[i].motionState.positions, trj.holonomic_path.points[i+1].motionState.positions);
      
      delta_theta += fabs(utility_.findDistanceBetweenAngles(last_theta, theta));
      
      last_theta = theta;
    }
    //////ROS_INFO("dist: %f delta_theta: %f", dist, delta_theta);

    double max_v = max_speed_linear / 100.0;
    double max_w = max_speed_angular / 100.0;

    // Estimate how long to execute positional and angular displacements based on max velocity
    double estimated_linear   = dist / max_v;
    double estimated_rotation = delta_theta / max_w;

    //////ROS_INFO("estimated_linear: %f estimated_rotation: %f", estimated_linear, estimated_rotation);

    T += (estimated_linear + estimated_rotation);
    if (T < 0.00001) T = 0.00001;
    double _1_T = 1.0 / T;

    // Orientation
    double A = orientation_.perform(trj);
    if (A < 0.00001) A = 0.00001;
    double _1_A = 1.0 / A;

    /*double v = sqrt( pow(trj.trajectory.points[0].velocities[0],2) + pow(trj.trajectory.points[1].velocities[1],2) );
    A_weight_ = v;*/

    // Minimum distance to any obstacle
    double D = cd_.min_dist_;
    if (D < 0.00001) D = 0.00001;
    double _1_D = 1.0 / D; // consider 1/D, not D
    
    //ROS_INFO("T: %f A: %f D: %f", T, A, D);

    // Update normalization for Time if necessary
    if(_1_T > _1_T_norm_)
    {
      _1_T_norm_ = _1_T;
    }
    if(_1_A > _1_A_norm_)
    {
      _1_A_norm_ = _1_A;
    }
    if(D > D_norm_)
    {
      D_norm_ = D;
    }

    // Normalize terms
    _1_T /= _1_T_norm_;
    _1_A /= _1_A_norm_;
    // D /= D_norm_;
    D /= D_norm_;
    
    //ROS_INFO("Normalized terms T: %f A: %f D: %f", T, A, D);

    // Weight terms
    // T_weight_ = 1.0;
    if (!ros::param::get("/ramp/eval_weight_T", T_weight_)) {
      // if fail to get the parameter
      T_weight_ = 1.0; // set it to the default
    }
    static bool is_set_T = false;
    if (!is_set_T) {
      ros::param::set("/ramp/eval_weight_T", T_weight_);
      is_set_T = true;
    }

    if (!ros::param::get("/ramp/eval_weight_A", A_weight_)) {
      // if fail to get the parameter
      A_weight_ = 0.005; // set it to the default
    }
    static bool is_set_A = false;
    if (!is_set_A) {
      ros::param::set("/ramp/eval_weight_A", A_weight_);
      is_set_A = true;
    }

    if (!ros::param::get("/ramp/eval_weight_D", D_weight_)) {
      // if fail to get the parameter
      D_weight_ = 15.0; // set it to the default
    }
    static bool is_set_D = false;
    if (!is_set_D) {
      ros::param::set("/ramp/eval_weight_D", D_weight_);
      is_set_D = true;
    }

    // T *= T_weight_;
    // A *= A_weight_;
    // D *= D_weight_; // this is wrong! (1.0)/(D * D_weight_)
    
    //ROS_INFO("Weighted terms T: %f A: %f D: %f", T, A, D);
    
    // Compute overall cost
    // cost = T_weight_ * T + A_weight_ * A + D_weight_ * _1_D;
    // cost += 0.00001; // cost cannot be zero, otherwise divided by zero

    result = 100000; // must be larger than infeasible
    result += T_weight_ * _1_T + A_weight_ * _1_A + D_weight_ * D;
  }

  else // infeasible
  {
    penalties = 1.0; // if infeasible, penalties cannot be zero
    //ROS_INFO("In else(infeasible)"); 
    
    // penalties += orientation_.getPenalty();
    
    ////////////ROS_INFO("trj.t_firstColl: %f", trj.t_firstCollision.toSec());

    // Q_coll_ = 1.0;
    if (!ros::param::get("/ramp/eval_weight_Qc", Q_coll_)) {
      // if fail to get the parameter
      Q_coll_ = 1.0; // set it to the default
    }
    static bool is_set_Qc = false;
    if (!is_set_Qc) {
      ros::param::set("/ramp/eval_weight_Qc", Q_coll_);
      is_set_Qc = true;
    }
    // Add the Penalty for being infeasible due to collision, at some point i was limiting the time to 10s, but i don't know why
    if(trj.t_firstCollision.toSec() > 0 && trj.t_firstCollision.toSec() < 9998)
    {
      ////ROS_INFO("In if t_firstCollision: %f", trj.t_firstCollision.toSec());
      ////ROS_INFO("Collision penalty: %f",(Q_coll_ / trj.t_firstCollision.toSec()));
      penalties += (Q_coll_ / trj.t_firstCollision.toSec());
    }
    else
    {
      ////ROS_INFO("Collision penalty (Full): %f",Q_coll_);
      // penalties += Q_coll_; // penalize more on collision later? This is wrong!
      penalties += (Q_coll_ / trj.t_firstCollision.toSec()); // enlarge factor should be large enough
    }
    double coll_time = trj.t_firstCollision.toSec(); // larger, better
    if (coll_time > coll_time_norm_) {
      coll_time_norm_ = coll_time;
    }
    coll_time /= coll_time_norm_;

    // If infeasible due to orientation change (i.e. no smooth curve)
    // If there is imminent collision, do not add this penalty (it's okay to stop and rotate)
    double _1_delta_theta;
    if(1 || orientation_infeasible_)// && !imminent_collision_)
    {
      //ROS_INFO("In if orientation_infeasible_: %f", orientation_.getDeltaTheta(trj));
      double delta_theta = orientation_.getDeltaTheta(trj) < 0.11 ? 0.1 : orientation_.getDeltaTheta(trj);
      if (delta_theta < 0.00001) delta_theta = 0.00001;
      _1_delta_theta = 1.0 / delta_theta;
      if (!ros::param::get("/ramp/eval_weight_Qk", Q_kine_)) {
        // if fail to get the parameter
        Q_kine_ = 10.0; // set it to the default
      }
      static bool is_set_Qk = false;
      if (!is_set_Qk) {
        ros::param::set("/ramp/eval_weight_Qk", Q_kine_);
        is_set_Qk = true;
      }
      //ROS_INFO("delta_theta: %f getDeltaTheta: %f Orientation penalty: %f", delta_theta, orientation_.getDeltaTheta(trj), Q_kine_ * (orientation_.getDeltaTheta(trj) / PI));
      if (_1_delta_theta > _1_A_norm_) {
        _1_A_norm_ = _1_delta_theta;
      }
      penalties += Q_kine_ * (delta_theta / A_norm_);
      _1_delta_theta /= _1_A_norm_;
    }

    penalties *= 100000.0; // the fitness of infeasible trajextory must be smaller than feasible trajectory
    result = Q_coll_* coll_time + Q_kine_ * _1_delta_theta;
  }

  ////////ROS_INFO("cost: %f penalties: %f", cost, penalties);
  // result = (1. / (cost + penalties));

  // if weights change, print them
  if (T_weight_ != last_T_weight_ ||
      A_weight_ != last_A_weight_ ||
      D_weight_ != last_D_weight_ ||
      Q_coll_   != last_Q_coll_   ||
      Q_kine_   != last_Q_kine_) {
        printf("weights have changed to: T = %.3lf, A = %.5lf, D = %.3lf, Qc = %.3lf, Qk = %.3lf\n",
          T_weight_, A_weight_, D_weight_, Q_coll_, Q_kine_);
        last_T_weight_ = T_weight_; last_A_weight_ = A_weight_; last_D_weight_ = D_weight_;
        last_Q_coll_   = Q_coll_;   last_Q_kine_   = Q_kine_;
      }

  ////////////ROS_INFO("performFitness time: %f", (ros::Time::now() - t_start).toSec());
  //ROS_INFO("Exiting Evaluate::performFitness");
} //End performFitness
