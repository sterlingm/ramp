#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(5), D_(1.5f), 
  cc_started_(false), transThreshold_(1./50.), num_cc_(0), L_(0.33), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), 
  stop_(false), moving_on_coll_(false)
{
  planningCycle_          = ros::Duration(1.f / 2.f);
  imminentCollisionCycle_ = ros::Duration(1.f / 10.f);
  generationsPerCC_       = controlCycle_.toSec() / planningCycle_.toSec();
}

Planner::~Planner() 
{
  if(h_traj_req_!= 0) {

    delete h_traj_req_;
    h_traj_req_= 0;
  }


  if(h_control_ != 0) 
  {
    delete h_control_;
    h_control_ = 0;
  }

  if(h_eval_req_ != 0) 
  {
    delete h_eval_req_;
    h_eval_req_ = 0;
  }
  
  if(modifier_!= 0) 
  {
    delete modifier_;  
    modifier_= 0;
  }
}





//TODO: Get this from parameters...
/** Transformation matrix of obstacle robot from base frame to world frame*/
void Planner::setOb_T_w_odom() 
{
  
  // Obstacle 1
  tf::Transform temp;
  tf::Vector3 pos(3.5f, 3.5, 0.f);
  temp.setRotation(tf::createQuaternionFromYaw(-3.f*PI/4.f));
  temp.setOrigin(pos);

  ob_T_w_odom_.push_back(temp);

 
  // Obstacle 2
  tf::Vector3 pos_two(0.f, 3.5, 0.f);
  temp.setOrigin(pos_two);
  temp.setRotation(tf::createQuaternionFromYaw(-3.f*PI/4.f));
  
  ob_T_w_odom_.push_back(temp);
} // End setOb_T_w_odom



/** This method determines what type of motion an obstacle has */
const MotionType Planner::findMotionType(const ramp_msgs::Obstacle ob) const 
{
  MotionType result;

  // Find the linear and angular velocities
  tf::Vector3 v_linear;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

  tf::Vector3 v_angular;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

  // Find magnitude of velocity vectors
  float mag_linear_t  = sqrt( tf::tfDot(v_linear, v_linear)   );
  float mag_angular_t = sqrt( tf::tfDot(v_angular, v_angular) );


  // Translation only
  // normally 0.0066 when idle
  if(mag_linear_t >= 0.0001 && mag_angular_t < 0.1) 
  {
    //ROS_INFO("Obstacle MotionType: Translation");
    result = MT_TRANSLATION;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.1) 
  {
    //ROS_INFO("Obstacle MotionType: Rotation");
    result = MT_ROTATION;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.1) 
  {
    //ROS_INFO("Obstacle MotionType: Translation and Rotation");
    result = MT_TRANSLATON_AND_ROTATION;
  }

  // Else, there is no motion
  else 
  {
    //ROS_INFO("Obstacle MotionType: None");
    result = MT_NONE;
  }

  return result;
} // End findMotionType




/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::RampTrajectory Planner::getPredictedTrajectory(const ramp_msgs::Obstacle ob, const tf::Transform tf) const 
{
  ramp_msgs::RampTrajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);
  

  // Now build a Trajectory Request 
  ramp_msgs::TrajectoryRequest tr;
    tr.request.path = getObstaclePath(ob, tf, motion_type);
    tr.request.type = PREDICTION;  // Prediction


  // Get trajectory
  if(h_traj_req_->request(tr)) 
  {
    result = tr.response.trajectory;
  }

  return result;
} // End getPredictedTrajectory






/** 
 *  This method returns a prediction for the obstacle's path. 
 *  The path is based on 1) the type of motion the obstacle currently has
 *  2) the duration that we should predict the motion for 
 */
const ramp_msgs::Path Planner::getObstaclePath(const ramp_msgs::Obstacle ob, const tf::Transform T_w_odom, const MotionType mt) const 
{
  ramp_msgs::Path result;

  std::vector<ramp_msgs::KnotPoint> path;

  //ROS_INFO("tf: (%f, %f, %f)", T_w_odom.getOrigin().getX(), T_w_odom.getOrigin().getY(), tf::getYaw(T_w_odom.getRotation()));

  /***********************************************************************
   Create and initialize the first point in the path
   ***********************************************************************/
  ramp_msgs::KnotPoint start;
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.x);
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.y);
  start.motionState.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.x);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.y);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.angular.z);
  
  //ROS_INFO("start before transform: %s", utility_.toString(start).c_str());

  /** Transform point based on the obstacle's odometry frame */
  // Transform the position
  tf::Vector3 p_st(start.motionState.positions.at(0), start.motionState.positions.at(1), 0); 
  tf::Vector3 p_st_tf = T_w_odom * p_st;

  //ROS_INFO("p_st: (%f, %f, %f)", p_st.getX(), p_st.getY(), p_st.getZ());
  
  start.motionState.positions.at(0) = p_st_tf.getX();
  start.motionState.positions.at(1) = p_st_tf.getY();
  start.motionState.positions.at(2) = utility_.displaceAngle(
      tf::getYaw(T_w_odom.getRotation()), start.motionState.positions.at(2));
  
  //ROS_INFO("start position after transform: %s", utility_.toString(start).c_str());
  
  // Transform the velocity
  std::vector<double> zero; zero.push_back(0); zero.push_back(0); 
  double teta = utility_.findAngleFromAToB(zero, start.motionState.positions);
  double phi = start.motionState.positions.at(2);
  double v = start.motionState.velocities.at(0);

  //ROS_INFO("teta: %f phi: %f v: %f", teta, phi, v);

  start.motionState.velocities.at(0) = v*cos(phi);
  start.motionState.velocities.at(1) = v*sin(phi);

  //ROS_INFO("start (position and velocity) after transform: %s", utility_.toString(start).c_str());


  if(v < 0) 
  {
    start.motionState.positions.at(2) = utility_.displaceAngle(start.motionState.positions.at(2), PI);
  }
  
  /***********************************************************************
   ***********************************************************************
   ***********************************************************************/

  // Push the first point onto the path
  path.push_back(start);

  if(mt == MT_NONE)
  {
    path.push_back(start);
  }

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(mt == MT_TRANSLATION) 
  {

    // Create the Goal Knotpoint
    ramp_msgs::KnotPoint goal;


    double theta = start.motionState.positions.at(2);
    double delta_x = cos(phi)*ob.odom_t.twist.twist.linear.x;
    double delta_y = sin(phi)*ob.odom_t.twist.twist.linear.x;
    //ROS_INFO("phi: %f theta: %f delta_x: %f delta_y: %f", phi, theta, delta_x, delta_y);
   

    ros::Duration predictionTime_(10.0f);
    // Get the goal position in the base frame
    tf::Vector3 ob_goal_b(start.motionState.positions.at(0) + (delta_x * predictionTime_.toSec()), 
                          start.motionState.positions.at(1) + (delta_y * predictionTime_.toSec()),
                          0);

    goal.motionState.positions.push_back(ob_goal_b.getX());
    goal.motionState.positions.push_back(ob_goal_b.getY());
    goal.motionState.positions.push_back(start.motionState.positions.at(2));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(0));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(1));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(2));

    //ROS_INFO("goal: %s", utility_.toString(goal.motionState).c_str());


    // Push goal onto the path
    path.push_back(goal);
  } // end if translation


  //std::cout<<"\nPath: "<<utility_.toString(utility_.getPath(path));
  result = utility_.getPath(path);
  return result; 
}






void Planner::sensingCycleCallback(const ramp_msgs::ObstacleList& msg)
{
  //ROS_INFO("In sensingCycleCallback");
  //ROS_INFO("msg: %s", utility_.toString(msg).c_str());

  ros::Time start = ros::Time::now();

  //Population pop_obs;

  // For each obstacle, predict its trajectory
  for(uint8_t i=0;i<msg.obstacles.size();i++)
  {

    RampTrajectory ob_temp_trj = getPredictedTrajectory(msg.obstacles.at(i), ob_T_w_odom_.at(i));
    if(ob_trajectory_.size() < i+1)
    {
      ob_trajectory_.push_back(ob_temp_trj);
    }
    else
    {
      ob_trajectory_.at(i) = ob_temp_trj;
    }

    //pop_obs.add(ob_temp_trj);
    //ROS_INFO("Time to get obstacle trajectory: %f", (ros::Time::now() - start).toSec());
    //ROS_INFO("ob_trajectory_: %s", ob_temp_trj.toString().c_str());
  } // end for

  //ROS_INFO("Time to generate obstacle trajectories: %f", (ros::Time::now() - start).toSec());
  ros::Time s = ros::Time::now();
  population_       = evaluatePopulation(population_);
  transPopulation_  = evaluatePopulation(transPopulation_);
  //ROS_INFO("Time to evaluate population: %f", (ros::Time::now() - s).toSec());
  ////ROS_INFO("Pop now: %s", population_.toString().c_str());
  ////ROS_INFO("Trans Pop now: %s", transPopulation_.toString().c_str());
  
  /*if(cc_started_)
  {
    RampTrajectory temp_mo = movingOn_.getSubTrajectoryPost(c_pc_ * planningCycle_.toSec());
    temp_mo.msg_.t_start = ros::Duration(0);
    temp_mo = evaluateTrajectory(temp_mo);
    moving_on_coll_ = !temp_mo.msg_.feasible;
  }*/

  //movingOn_ = evaluateTrajectory(movingOn_);
  //ROS_INFO("movingOn_ Feasible: %s", movingOn_.msg_.feasible ? "True" : "False");

  sc_durs_.push_back( ros::Time::now() - start );
  
  if(cc_started_)
  {
    sendPopulation(transPopulation_);
  }
  else
  {
    sendPopulation(population_);
  }

  //sendPopulation(pop_obs);

  ROS_INFO("Time in sensing cycle: %f", (ros::Time::now() - start).toSec());
  //ROS_INFO("Exiting sensingCycleCallback");
}




















const std::vector<Path> Planner::getRandomPaths(const MotionState init, const MotionState goal) 
{
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) 
  {
    
    // Create the path with the start and goal
    Path temp_path = getRandomPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getRandomPaths


const std::vector<Path> Planner::getAdjustedPaths(const MotionState init, const MotionState goal) 
{
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path = getAdjustedPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getAdjustedPaths




/*
 * Generate a path from s to g with completely random intermediate knot points (no constraints imposed)
 */
const Path Planner::getRandomPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  

  // Each trajectory will have a random number of knot points
  // Put a max of 3 knot points for practicality...
  uint8_t n = (rand() % 3)+1;

  // Create n knot points 
  for(uint8_t i=0;i<n;i++) 
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Push on velocity values?
    // 
    
    result.addBeforeGoal(ms_temp);
  }

  return result;
} // End getRandomPath



/*
 * Return true if the MotionState ms satisfies constraints on Knot Points to be added to Path p
 */
const bool Planner::validKPForPath(const MotionState ms, const Path p) const
{
  bool result=true;

  // Check that ms_temp has a distance > L from all other knot points
  for(uint8_t i=0;i<p.size();i++)
  {
    if( sqrt( pow(ms.msg_.positions.at(0) - p.at(i).motionState_.msg_.positions.at(0), 2) +
              pow(ms.msg_.positions.at(1) - p.at(i).motionState_.msg_.positions.at(1), 2) ) < L_)
    {
      result = false;
      i = p.size();
    }
  }

  return result;
} // End validForKP



/*
 * Generate a path from s to g with intermediate knot points that are constrained
 */
const Path Planner::getAdjustedPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  
  // Each trajectory will have a random number of knot points
  // Put a max of 3 knot points for practicality...
  uint8_t n = (rand() % 3)+1;

  while(result.size() < (n+2))
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Check that it satisfies any constraints on KPs
    if(validKPForPath(ms_temp, result))
    {
      result.addBeforeGoal(ms_temp);
    }
  } // end while

  return result;
} // End getAdjustedPath




const Population Planner::getPopulation( const MotionState init, const MotionState goal, const bool random)
{
  Population result;

  // Set the size
  result.maxSize_ = populationSize_;

  // Get some random paths
  std::vector<Path> paths = random ?  getRandomPaths  (init, goal)  : 
                                      getAdjustedPaths(init, goal)  ;

  // Get trajectories for the paths
  std::vector<RampTrajectory> trajecs = getTrajectories(paths);

  // Add each trajectory to the population
  // Use add over replaceAll in case of sub-populations
  for(uint8_t i=0;i<trajecs.size();i++) 
  {
    result.add(trajecs.at(i));
  }

  // Create sub-pops if enabled
  if(subPopulations_) 
  {
    result.createSubPopulations();
  }

  // Evaluate the population 
  result = evaluatePopulation(result);

  ////ROS_INFO("Exiting Planner::getRandomPopulation");
  return result;
} // End getPopulation



const uint8_t Planner::getIndexStartPathAdapting(const RampTrajectory t) const 
{
  /*//ROS_INFO("In Planner::getIndexStartPathAdapting");
  //ROS_INFO("t transTraj.size(): %i", (int)t.transitionTraj_.trajectory.points.size());
  //ROS_INFO("# of curves: %i", (int)t.msg_.curves.size());*/
  uint8_t result;
  bool    has_curve = t.msg_.curves.size() > 0;

  if(t.transitionTraj_.trajectory.points.size() > 0) {
    //ROS_INFO("In t.transitionTraj_.trajectory.points.size() > 0");
    result = t.transitionTraj_.i_knotPoints.size();
  }
  else if(t.msg_.curves.size() > 1 && t.transitionTraj_.trajectory.points.size() == 0)
  {
    result = 3;
  }
  else if(has_curve && t.msg_.curves.at(0).u_0 == 0) {
    result = 2;
  }
  else {
    result = 1;
  }

  // If the first part is just self-rotation to correct orientation,
  // add 1 to result
  if(t.msg_.i_knotPoints.size() > 1 && utility_.positionDistance( 
                                        t.msg_.trajectory.points.at( t.msg_.i_knotPoints.at(1)).positions,
                                        t.msg_.trajectory.points.at( t.msg_.i_knotPoints.at(0)).positions) 
      < 0.001)
  {
    //ROS_WARN("Adding 1 to result because first two position are the same, indicating a rotation to satisfy orientation");
    result++;
  }

  //ROS_INFO("getIndexStartPathAdapting returning: %i", result);
  return result;
}




const uint8_t Planner::getNumThrowawayPoints(const RampTrajectory traj, const ros::Duration dur) const {
  uint8_t result = 1;

  // For each knot point,
  // Start at 2 because that is the end of the first bezier curve
  for(uint8_t i_kp=getIndexStartPathAdapting(traj);
      i_kp<traj.msg_.i_knotPoints.size();
      i_kp++) 
  {
    ////ROS_INFO("i_kp: %i", (int)i_kp);
    
    // Only adapt the best trajectory
    // TODO: Make this method not do a loop
    if(traj.equals(transPopulation_.getBest()))
    {
      // Get the knot point 
      trajectory_msgs::JointTrajectoryPoint point = traj.msg_.trajectory.points.at( 
                                                      traj.msg_.i_knotPoints.at(i_kp));
      ////ROS_INFO("point: %s", utility_.toString(point).c_str());

      // Compare the durations
      if( (dur > point.time_from_start) || 
          (fabs(dur.toSec() - point.time_from_start.toSec()) < 0.0001) ) 
      {
        ////ROS_INFO("Past KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
        result++;
      }
      else {
        ////ROS_INFO("Behind KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
        break;
      }
    } // end if best trajectory
    else
    {
      //ROS_INFO("Not best trajectory in transition population, only removing first knot point (startPlanning_)");
    }
  } // end for


  return result;
}

/** 
 * This method updates all the paths with the current configuration 
 * For each knot point in a path, it will remove the knot point if
 * its time_from_start is <= the Duration argument
 * */
const std::vector<Path> Planner::adaptPaths(const Population pop, const MotionState start, ros::Duration dur) const {
  //ROS_INFO("In Planner::adaptPaths");
  //ROS_INFO("pop.paths.size(): %i", (int)pop.paths_.size());
  //ROS_INFO("dur.toSec(): %f", dur.toSec());
  std::vector<Path> result;

  // Check that time has passed
  if(dur.toSec() > 0) 
  {


    // For each trajectory
    for(uint8_t i=0;i<pop.size();i++) {
      ////ROS_INFO("Path: %s", pop.paths_.at(i).toString().c_str());
      ////ROS_INFO("Get Path: %s", pop.get(i).getPath().toString().c_str());
      Path temp = pop.paths_.at(i);

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=getNumThrowawayPoints(pop.get(i), dur);
      ////ROS_INFO("throwaway: %i", (int)throwaway);

      
      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= pop.paths_.at(i).size() ) 
      { 
        ////ROS_INFO("Decrementing throwaway");
        throwaway = pop.paths_.at(i).size()-1;
      }


      // Print the knot points being removed
      /*for(int c=0;c<throwaway;c++) {
        //ROS_INFO("\nRemoving point: %s", (*(pop.paths_.at(i).all_.begin()+c)).toString().c_str());
      }*/

      // Erase the amount of throwaway points (points we have already passed)
      temp.all_.erase( 
          temp.all_.begin(), 
          temp.all_.begin()+throwaway );

      // Insert the new starting configuration
      temp.all_.insert( temp.all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      temp.start_ = start;
      ////ROS_INFO("After adapting Path: %s", temp.toString().c_str());

      result.push_back(temp);
    } // end outer for
  } // end if dur > 0

  //ROS_INFO("Exiting adaptPaths");
  return result;
} // End adaptPaths





// 1 if before curve, 2 if on curve, 3 if past curve 
// TODO: Check for the 2nd segment as well?
const int Planner::estimateIfOnCurve(const MotionState ms, const ramp_msgs::BezierCurve curve) const {
  //ROS_INFO("In estimateIfOnCurve");
  //ROS_INFO("ms: %s", ms.toString().c_str());
  //ROS_INFO("curve: %s", utility_.toString(curve).c_str());

  double x = ms.msg_.positions.at(0);
  double y = ms.msg_.positions.at(1);

  bool xSlope     = (curve.segmentPoints.at(1).positions.at(0) - curve.segmentPoints.at(0).positions.at(0) > 0);
  bool xSlopeTwo  = (curve.segmentPoints.at(2).positions.at(0) - curve.segmentPoints.at(1).positions.at(0) > 0);
  bool ySlope     = (curve.segmentPoints.at(1).positions.at(1) - curve.segmentPoints.at(0).positions.at(1) > 0);
  bool ySlopeTwo  = (curve.segmentPoints.at(2).positions.at(1) - curve.segmentPoints.at(1).positions.at(1) > 0);
  
  bool xSegOne =  xSlope ?  (x >= curve.controlPoints.at(0).positions.at(0)) &&
                            (x <= curve.controlPoints.at(1).positions.at(0)) :
                            (x <= curve.controlPoints.at(0).positions.at(0)) &&
                            (x >= curve.controlPoints.at(1).positions.at(0));

  bool xSegTwo =  xSlopeTwo ?   (x >= curve.controlPoints.at(1).positions.at(0)) &&
                                (x <= curve.controlPoints.at(2).positions.at(0)) :
                                (x <= curve.controlPoints.at(1).positions.at(0)) &&
                                (x >= curve.controlPoints.at(2).positions.at(0));

  
  bool ySegOne =  ySlope ?  (y >= curve.controlPoints.at(0).positions.at(1)) &&
                            (y <= curve.controlPoints.at(1).positions.at(1)) :
                            (y <= curve.controlPoints.at(0).positions.at(1)) &&
                            (y >= curve.controlPoints.at(1).positions.at(1));

  bool ySegTwo =  ySlopeTwo ?   (y >= curve.controlPoints.at(1).positions.at(1)) &&
                                (y <= curve.controlPoints.at(2).positions.at(1)) :
                                (y <= curve.controlPoints.at(1).positions.at(1)) &&
                                (y >= curve.controlPoints.at(2).positions.at(1));


  ////ROS_INFO("xSlope: %s xSlopeTwo: %s ySlope: %s ySlopeTwo: %s", xSlope ? "True" : "False", xSlopeTwo ? "True" : "False", ySlope ? "True" : "False", ySlopeTwo ? "True" : "False"); 
  ////ROS_INFO("xSegOne: %s xSegTwo: %s ySegOne: %s ySegTwo: %s", xSegOne ? "True" : "False", xSegTwo ? "True" : "False", ySegOne ? "True" : "False", ySegTwo ? "True" : "False");

  bool xGood = (xSegOne || xSegTwo);
  bool yGood = (ySegOne || ySegTwo);

  if(xGood && yGood) {
    ////ROS_INFO("Returning 2 (on curve)");
    return 2;
  }
  
  // Check if past segment 1
  bool xPastOne = xSlope ?  x > curve.controlPoints.at(1).positions.at(0) :
                            x < curve.controlPoints.at(1).positions.at(0) ; 
  // Check if past segment 1
  bool yPastOne = ySlope ?  y > curve.controlPoints.at(1).positions.at(1) :
                            y < curve.controlPoints.at(1).positions.at(1) ; 

  ////ROS_INFO("xPastOne: %s yPastOne: %s", xPastOne ? "True" : "False", yPastOne ? "True" : "False");
  // If past segment 1, check if past segment 2
  if(xPastOne && yPastOne)
  { 
  
    bool xPastTwo = xSlopeTwo ?   x > curve.controlPoints.at(2).positions.at(0) :
                                  x < curve.controlPoints.at(2).positions.at(0) ; 
    bool yPastTwo = ySlopeTwo ?   y > curve.controlPoints.at(2).positions.at(1) :
                                  y < curve.controlPoints.at(2).positions.at(1) ; 

    ////ROS_INFO("xPastTwo: %s yPastTwo: %s", xPastTwo ? "True" : "False", yPastTwo ? "True" : "False");
    if(xPastTwo || yPastTwo)
    {
      ////ROS_INFO("Returning 3 (after curve)");
      return 3;
    }
  } // end if past segment 1

  // Else, robot has not reached curve, return 1
  //ROS_INFO("Returning 1");
  return 1;
}



const ramp_msgs::BezierCurve Planner::handleCurveEnd(const RampTrajectory traj) const 
{
  ramp_msgs::BezierCurve result;

  // If there are 2 curves, set the next one up 
  // Currently, the adaptive control cycles always end at
  // the start of the next curve so we can safely assume the position should be 0
  // If the control cycles will ever occur in the middle of a 2nd curve, we will have to calculate u_0
  if(traj.msg_.curves.size() > 1) 
  {
    result = traj.msg_.curves.at(1);
    ////ROS_INFO("Curve 0 has ended, new curve: %s", utility_.toString(result).c_str());
    if(estimateIfOnCurve(startPlanning_, result) == 2) {
      ////ROS_INFO("Adding .000001");
      result.u_0 += 0.000001;
      result.ms_begin = startPlanning_.msg_;
    }
    else 
    {
      result.u_0 += 0.;
    }
  }

  // Else if there was only 1 curve, check the path and set the segment points for the next one
  else
  {
    if(traj.path_.size() > 3)
    {
      ////ROS_INFO("traj.path: %s", traj.path_.toString().c_str());
      if(traj.path_.size() == 3)
      {

      } // end if size==3
      if(traj.path_.size() < 4)
      {
        //ROS_ERROR("traj.path.size(): %i", (int)traj.path_.size());
      }
      result.segmentPoints.push_back(traj.path_.at(1).motionState_.msg_);
      result.segmentPoints.push_back(traj.path_.at(2).motionState_.msg_);
      result.segmentPoints.push_back(traj.path_.at(3).motionState_.msg_);
    } // end if path.size() > 2
  } // end if only one curve

  return result;
} // End handleCurveEnd



const double Planner::updateCurvePos(const RampTrajectory traj, const ros::Duration d) const 
{
  ////ROS_INFO("In Planner::updateCurvePos");
  ////ROS_INFO("d: %f", d.toSec());
  
  ramp_msgs::BezierCurve curve = traj.msg_.curves.at(0); 
  double result = curve.u_0;

  // if u_0==0 then estimateIfOnCurve returned 2 - already on curve
  if(curve.u_0 < 0.00001) 
  {
    ////ROS_INFO("In if curve.u_0==0");

    // Get the time at the start of the curve
    double t_s0 = traj.msg_.trajectory.points.at(
        traj.msg_.i_knotPoints.at(1)).time_from_start.toSec();
    ////ROS_INFO("t_s0: %f", t_s0);

    // t = the time spent moving on the curve
    double t = d.toSec() - t_s0;
    ////ROS_INFO("t: %f", t);
    
    // Previously was subtracting 1 from index. Not sure why that worked, but keep in mind if future issues arise
    // Check if index >= size because of rounding errors
    int index = floor(t*10)+1 >= curve.u_values.size() ? floor(t*10) : floor(t*10)+1;
    ////ROS_INFO("index: %i u_values.size: %i", index, (int)curve.u_values.size());
    if(index < curve.u_values.size())
    {
      ////ROS_INFO("u_values[%i]: %f", index, curve.u_values.at(index));
    }

    if(t < 0.0001)
    {
      result = 0.0001;
    }
    else if(index >= curve.u_values.size())
    {
      ////ROS_INFO("index: %i curve.u_values.size(): %i, setting result to 1.1", index, (int)curve.u_values.size());
      result = 1.1;
    }
    else
    {

      ////ROS_INFO("t: %f Adding %f", t, (t*curve.u_dot_0));
      
      //result += t * curve.u_dot_0;
      result = curve.u_values.at(index);
    }
  } // end if already on curve

  // Else, u_0 > 0, simply add to u_0
  else {
    ////ROS_INFO("Else curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
    result += curve.u_dot_0 * d.toSec();
  }

  ////ROS_INFO("Exiting Planner::updateCurvePos");
  return result;
} // End updateCurvePos


/** Updates the curve u_0 and ms_begin */
// TODO: Only need to update the bestTrajec's curve
const std::vector<ramp_msgs::BezierCurve> Planner::adaptCurves(const Population pop, const MotionState ms, const ros::Duration d) const 
{
  ////ROS_INFO("In Planner::adaptCurves");

  std::vector<ramp_msgs::BezierCurve> result;
  ramp_msgs::BezierCurve blank;

  // Go through each trajectory 
  for(uint16_t i=0;i<pop.size();i++) 
  {
    //ROS_INFO("Curve %i", (int)i);
    
    // If the trajectory has a curve
    // Don't check for best trajec here b/c we want to push on the same curve if we haven't moved on it, not a blank 
    // curve
    if(pop.get(i).msg_.curves.size() > 0) 
    {
      ////ROS_INFO("In if trajectory has curve");

      // Set curve
      ramp_msgs::BezierCurve curve = pop.get(i).msg_.curves.size() > 1 ? pop.get(i).msg_.curves.at(1) :
                                                                        pop.get(i).msg_.curves.at(0) ;
      ////ROS_INFO("Set curve to: %s", utility_.toString(curve).c_str());

      ////ROS_INFO("pop.getBestIndex: %i", (int)pop.calcBestIndex());
      // If moving on this curve, update u
      if( i == pop.calcBestIndex() && 
            (curve.u_0 > 0. ||
             estimateIfOnCurve(ms, curve) == 2))
      {
        //ROS_INFO("Moving on this curve");

        // Get the new u_0 value
        curve.u_0 = updateCurvePos(pop.get(i), d);

        // Set the new ms_begin
        curve.ms_begin = ms.msg_;
      }  //end if moving on curve
      else if(i != pop.calcBestIndex())
      {
        //ROS_INFO("Not moving on curve, erase it and start with new segment points");
        curve = blank;
      }
      else
      {
        //ROS_INFO("Curve is for best trajectory, but not yet moving on curve");
      }

     
      /* Separate checking if on the curve and if done with curve
           because we could be done before ever incrementing u_0 */
      // Check if done with current curve
      if( i == pop.calcBestIndex() && (curve.u_0 > curve.u_target || estimateIfOnCurve(ms, curve) == 3) )
      {
        //ROS_INFO("Done with curve, u_0: %f", curve.u_0);
        curve = handleCurveEnd(pop.get(i));
      } // end if done with 1st curve
      else
      {
        //ROS_INFO("Not done with curve");
      }

      //ROS_INFO("Curve after adapting: %s", utility_.toString(curve).c_str());
      result.push_back(curve);
    } // end if trajectory has curve

    // Else if there is no curve, push on a blank one
    else {
      ////ROS_INFO("No curve");
      result.push_back(blank);
    } // end else no curve
  } // end for

  //ROS_INFO("Exiting Planner::adaptCurves");
  return result;
} // End adaptCurves




/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
// Not const because it calls requestTrajectory which can call getIRT
const Population Planner::adaptPopulation(const Population pop, const MotionState ms, const ros::Duration d) 
{
  ////ROS_INFO("In adaptPopulation");
  ////ROS_INFO("pop: %s", pop.toString().c_str());
  ////ROS_INFO("startPlanning_: %s \nduration: %f", startPlanning_.toString().c_str(), d.toSec());
  Population result = pop;
  
  ////ROS_INFO("Before adaptPaths, paths.size(): %i", (int)pop.paths_.size());
  ////ROS_INFO("Before adaptPaths, paths.size(): %i", (int)result.paths_.size());

  // Find how long we've been moving on the curve - how long between CCs minus the start of the curve
  //ros::Duration curveD = ros::Duration(d.toSec() - transPopulation_.getBest().msg_.curves.at(0).controlPoints.at(0).time);
  ////ROS_INFO("curveD: %f", curveD.toSec());
 
  // Adapt the paths and curves
  std::vector<Path> paths                     = adaptPaths  (pop, ms, d);
  std::vector<ramp_msgs::BezierCurve> curves  = adaptCurves (pop, ms, d);

  result.paths_ = paths;

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  /*//ROS_INFO("pop.calcBestIndex(): %i", pop.calcBestIndex());
  //ROS_INFO("paths.size(): %i", (int)paths.size());
  //ROS_INFO("curves.size(): %i", (int)curves.size());*/
  // For each path, get a trajectory
  for(uint16_t i=0;i<pop.size();i++) {
    RampTrajectory temp, tempTraj = pop.get(i);
    ////ROS_INFO("Getting trajectory %i", (int)i);
      
    std::vector<ramp_msgs::BezierCurve> c;
    c.push_back(curves.at(i));

    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(paths.at(i), c);
    tr.request.segments = 2;

    /* Get the trajectory */
    temp = requestTrajectory(tr, result.get(i).msg_.id);

    ////ROS_INFO("Temp before: %s", temp.toString().c_str());
    temp = temp.concatenate(pop.get(i), 4);
    ////ROS_INFO("Temp now: %s", temp.toString().c_str());

    // Set temporary evaluation results - need to actually call requestEvaluation to get actual fitness
    temp.msg_.fitness   = result.get(i).msg_.fitness;
    temp.msg_.feasible  = result.get(i).msg_.feasible;
    temp.msg_.t_start   = ros::Duration(t_fixed_cc_);

    ////ROS_INFO("Finished adapting trajectory %i, t_start: %f", i, temp.msg_.t_start.toSec());

    // Push onto updatedTrajecs
    updatedTrajecs.push_back(temp);
  } // end for

  ////ROS_INFO("updatedTrajecs size: %i", (int)updatedTrajecs.size());
  // Replace the population's trajectories_ with the updated trajectories
  result.replaceAll(updatedTrajecs);
  
  ////ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  ////ROS_INFO("Exiting adaptPopulation");

  return result;
} // End adaptPopulation





// TODO: Clean up
/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierCurve> curves, const int id) const 
{
  //ROS_INFO("In buildTrajectoryRequest");
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.type           = PARTIAL_BEZIER;

  // If path size > 2, assign a curve
  if(path.size() > 2) {
    //ROS_INFO("In if path.size() > 2)");

    // If it's the first time getting a curve 
    if(curves.size() == 0 || curves.at(0).segmentPoints.size() == 0) 
    {
      if(path.size() > 2) 
      {
        //ROS_INFO("In temp curve");
        ramp_msgs::BezierCurve temp;
        
        temp.segmentPoints.push_back( path.all_.at(0).motionState_.msg_ );
        temp.segmentPoints.push_back( path.all_.at(1).motionState_.msg_ );
        temp.segmentPoints.push_back( path.all_.at(2).motionState_.msg_ );
        
        result.request.bezierCurves.push_back(temp);
      }
    }
    else 
    {
      //ROS_INFO("In else if path.size < 3");
      result.request.bezierCurves = curves;
    } // end else
  } // end if


  //ROS_INFO("Exiting Planner::buildTrajectoryRequest");
  return result;
} // End buildTrajectoryRequest


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const int id) const 
{
  std::vector<ramp_msgs::BezierCurve> curves;
  return buildTrajectoryRequest(path, curves, id);
}





/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory   = trajec.msg_;
  if(population_.size() > 0)
  {
    result.request.currentTheta = population_.getBest().getDirection();
  }
  else
  {
    result.request.currentTheta = latestUpdate_.msg_.positions.at(2);
  }

  for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    result.request.obstacle_trjs.push_back(ob_trajectory_.at(i).msg_);
  }

  return result;
} // End buildEvaluationRequest





/** This method initializes the T_w_odom_ transform object */
void Planner::setT_base_w(std::vector<double> base_pos) {
  T_w_odom_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_w_odom_.setOrigin(  tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
const unsigned int Planner::getIRT() { return i_rt++; }




/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) 
{
  //ROS_INFO("In imminentCollisionCallback");
  std_msgs::Bool ic;

  double dist_threshold = 0.45;

  for(uint8_t i=0;i<ob_trajectory_.size();i++)
  {
    
    if(ob_trajectory_.at(i).msg_.trajectory.points.size() > 0)
    {
      trajectory_msgs::JointTrajectoryPoint ob = ob_trajectory_.at(i).msg_.trajectory.points.at(0);
      double dist = utility_.positionDistance(ob.positions, latestUpdate_.msg_.positions);

      //ROS_WARN("latestUpdate_: %s\nob_point: %s", latestUpdate_.toString().c_str(), utility_.toString(ob).c_str());

      //if(!movingOn_.msg_.feasible &&
      if(moving_on_coll_ &&
          dist < dist_threshold)
      {
        //ROS_WARN("Imminent Collision Robot: %i dist: %f", id_, dist);
        //ROS_WARN("Obstacle trajectory: %s", ob_trajectory_.at(i).toString().c_str());

        ic.data = true;
        break;
      }

      else 
      {
        //ROS_INFO("No imminent collision, dist: %f", dist);
        //ROS_INFO("startPlanning: %s", startPlanning_.toString().c_str());
        ic.data = false;
      }
    }
    else
    {
      ic.data = false;
    }
  } // end for

  h_control_->sendIC(ic);

  //std::cout<<"\nAfter imminentCollisionCycle_\n";
}




/** 
 * Sets the latest update member
 * and transformes it by T_base_w because 
 * updates are relative to odometry frame
 * */
void Planner::updateCallback(const ramp_msgs::MotionState& msg) {
  ////ROS_INFO("In Planner::updateCallback");

 
  if(msg.positions.size() < 3 ||
     msg.velocities.size() < 3 ||
     msg.accelerations.size() < 3 )
  { 
    ROS_ERROR("Odometry message from ramp_control does not have all DOFs: %s", 
        utility_.toString(msg).c_str());
  }
  else {
    latestUpdate_ = msg;

    // Transform configuration from odometry to world coordinates
    latestUpdate_.transformBase(T_w_odom_);

    // Set proper velocity values
    latestUpdate_.msg_.velocities.at(0) = msg.velocities.at(0) * 
                                          cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.velocities.at(1) = msg.velocities.at(0) * 
                                          sin(latestUpdate_.msg_.positions.at(2));

    // Set proper acceleration values
    latestUpdate_.msg_.accelerations.at(0) = msg.accelerations.at(0) * 
                                             cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.accelerations.at(1) = msg.accelerations.at(0) * 
                                             sin(latestUpdate_.msg_.positions.at(2));

    ////ROS_INFO("New latestUpdate_: %s", latestUpdate_.toString().c_str());
  } // end else
  
  ////ROS_INFO("Exiting Planner::updateCallback");
} // End updateCallback






/** This method sets random values for the position vector of ms */
const MotionState Planner::randomizeMSPositions(const MotionState ms) const {
  MotionState result = ms;
  result.msg_.positions.clear();

  for(unsigned int i=0;i<ranges_.size();i++) 
  {
    result.msg_.positions.push_back(ranges_.at(i).random());
  }

  return result;
} // End randomizeMotionState


/****************************************************
 ************** Initialization Methods **************
 ****************************************************/



void Planner::initStartGoal(const MotionState s, const MotionState g) {
  start_  = s;
  goal_   = g; 

  m_cc_ = start_;
  startPlanning_ = start_;

  latestUpdate_ = start_;
}


/** Initialize the handlers and allocate them on the heap */
void Planner::init(const uint8_t i, const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r, const int population_size, const bool sub_populations, const std::vector<tf::Transform> ob_T_odoms, const double trans_delta_t, const int gens_before_cc, const double t_pc_rate, const double t_fixed_cc, const bool errorReduction) 
{
  ////ROS_INFO("In Planner::init");

  // Set ID
  id_ = i;

  // Initialize the handlers
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
  modifier_   = new Modifier(h, num_ops_);

  // Initialize the timers, but don't start them yet
  controlCycle_       = ros::Duration(t_fixed_cc);
  controlCycleTimer_  = h.createTimer(ros::Duration(controlCycle_), 
                                     &Planner::controlCycleCallback, this);
  controlCycleTimer_.stop();

  planningCycle_      = ros::Duration(1.f/t_pc_rate);
  //planningCycleTimer_ = h.createTimer(ros::Duration(planningCycle_), 
  //                                    &Planner::planningCycleCallback, this);
  planningCycleTimer_.stop();

  imminentCollisionTimer_ = h.createTimer(ros::Duration(imminentCollisionCycle_), 
                                          &Planner::imminentCollisionCallback, this);
  imminentCollisionTimer_.stop();



  // Set the ranges vector
  ranges_ = r;

  // Initialize the start and goal
  initStartGoal(s, g);

  // Set the base transformation
  setT_base_w(start_.msg_.positions);

  //setOb_T_w_odom();

  // Set misc members
  populationSize_       = population_size;
  subPopulations_       = sub_populations;
  ob_T_w_odom_          = ob_T_odoms;
  trans_delta_t_        = trans_delta_t;
  generationsBeforeCC_  = gens_before_cc;
  t_fixed_cc_           = t_fixed_cc;
  errorReduction_       = errorReduction;
  generationsPerCC_     = controlCycle_.toSec() / planningCycle_.toSec();
} // End init






/** Place code to seed population here */
void Planner::seedPopulation() {

  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(2.);
  kp.motionState.positions.push_back(2.);
  kp.motionState.positions.push_back(0.785); // 26 degrees 
  
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  
  ramp_msgs::KnotPoint kp1;
  
  kp1.motionState.positions.push_back(3.);
  kp1.motionState.positions.push_back(0.);
  kp1.motionState.positions.push_back(PI/4);
  
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all;
  all.push_back(start_);
  all.push_back(kp);
  //all.push_back(kp1);
  all.push_back(goal_);

  Path p1(all);


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(0.);
  kp2.motionState.positions.push_back(3.);
  kp2.motionState.positions.push_back(PI/2.);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(start_);
  all2.push_back(kp2);
  all2.push_back(goal_);

  Path p2(all2);

  ramp_msgs::KnotPoint kp3;
  
  kp3.motionState.positions.push_back(2.f);
  kp3.motionState.positions.push_back(0.f);
  kp3.motionState.positions.push_back(0.f);
  
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  std::vector<KnotPoint> all3;
  all3.push_back(start_);
  all3.push_back(kp3);
  all3.push_back(goal_);

  Path p3(all3);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p2);
  paths.push_back(p3);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  population_.clear();
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    trajec = requestEvaluation(trajec);
    //ROS_INFO("Seede trajec: %s", trajec.toString().c_str());
    population_.add(trajec); 
  
  } // end for
  /************************************/

} // End seedPopulation



/** Will seed population with a straight-line trajectory to the goal */
void Planner::seedPopulationTwo() {


  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(0.707); // 80 degrees 
  

  std::vector<KnotPoint> all;
  all.push_back(startPlanning_);
  all.push_back(kp);
  all.push_back(goal_);

  Path p1(all);


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(1);
  kp2.motionState.positions.push_back(-2);
  kp2.motionState.positions.push_back(2.21431);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(startPlanning_);
  all2.push_back(kp2);
  all2.push_back(goal_);

  Path p2(all2);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p2);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    new_pop.push_back(evaluateTrajectory(trajec));
  
  } // end for
  /************************************/

  population_.replaceAll(new_pop);  
} // End seedPopulationTwo


const std::vector<MotionState> Planner::setMi(const RampTrajectory trj_current) const 
{
  //ROS_INFO("In Planner::setMi");
  //ROS_INFO("trj_current: %s", trj_current.toString().c_str());
  std::vector<MotionState> result;
  
 
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) {
    MotionState temp = movingOn_.getPointAtTime(planningCycle_.toSec()*(i+1));
    
    result.push_back(temp);

    //ROS_INFO("m_i[%i]: %s", i, temp.toString().c_str());
  } // end for


  //ROS_INFO("Exiting Planner::setMi");
  return result;
} // End setMi




/** Pass in entire RampTrajectory because we need the path info */
const ramp_msgs::BezierCurve Planner::replanCurve(const RampTrajectory trajec, const MotionState ms_start) const 
{
  //ROS_INFO("In Planner::replanCurve");
  ramp_msgs::BezierCurve result = trajec.msg_.curves.at(0);

  // Get length of original curve's first segment
  double delta_x = trajec.path_.all_.at(1).motionState_.msg_.positions.at(0) - result.segmentPoints.at(0).positions.at(0);
  double delta_y = trajec.path_.all_.at(1).motionState_.msg_.positions.at(1) - result.segmentPoints.at(0).positions.at(1);
  double l = sqrt( pow(delta_x, 2) + pow(delta_y, 2) );

  double theta = ms_start.msg_.positions.at(2);

  double x = result.segmentPoints.at(0).positions.at(0) + l*cos(theta);
  double y = result.segmentPoints.at(0).positions.at(1) + l*sin(theta);

  //ROS_INFO("delta_x: %f delta_y: %f l: %f theta: %f x: %f y: %f", delta_x, delta_y, l, theta, x, y);
  result.segmentPoints.at(1).positions.at(0) = x;
  result.segmentPoints.at(1).positions.at(1) = y;
  result.controlPoints.clear();
  result.ms_initialVA.velocities.clear();
  result.ms_initialVA.accelerations.clear();
  result.u_0 = 0;
  result.u_dot_0 = 0;
  result.u_values.clear();
  result.u_dot_max = 0.;
  result.u_target = 0.;
 



  return result;
}


const RampTrajectory Planner::replanTrajec(const RampTrajectory trajec, const MotionState ms_start) 
{
  //ROS_INFO("In Planner::replanTrajec");

  RampTrajectory result = trajec;
  //ROS_INFO("After setting result");
  //ROS_INFO("Test my ID: pop best ID: %i", population_.getBest().msg_.id);
  //ROS_INFO("result: %s", result.toString().c_str());
  //ROS_INFO("result.curves.size: %i", (int)result.msg_.curves.size());

  result.path_.start_ = ms_start;
  //ROS_INFO("ms_start: %s", ms_start.toString().c_str());

  result.path_.all_.erase( result.path_.all_.begin() );
  result.path_.all_.insert( result.path_.all_.begin(), ms_start);

  double v = sqrt(  pow( ms_start.msg_.velocities.at(0), 2) + 
                    pow( ms_start.msg_.velocities.at(1), 2) );
  //ROS_INFO("v: %f", v);

  // Replan the curve if it's the best trajectory
  if( trajec.equals(population_.getBest())  && 
      result.msg_.curves.size() > 0         && 
      v > 0.0001                            && 
      result.msg_.curves.at(0).u_0 < 0.001      ) 
  {
    result.msg_.curves.at(0) = replanCurve( trajec, ms_start );
    result.path_.all_.erase(result.path_.all_.begin()+1);
    MotionState m(result.msg_.curves.at(0).segmentPoints.at(1));
    result.path_.all_.insert(result.path_.all_.begin()+1, m);
   
  }
  else 
  {
    //ROS_INFO("Not replanning curve for trajec id: %i", trajec.msg_.id);
    //ROS_INFO("v: %f curves.size(): %i", v, (int)result.msg_.curves.size());
    if(result.msg_.curves.size() > 0) 
    {
      //ROS_INFO("curve.u_0: %f", result.msg_.curves.at(0).u_0);
    }
  }

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(result.path_, result.msg_.curves, trajec.msg_.id);

  result = requestTrajectory(tr, result.msg_.id);

  //ROS_INFO("Replanned Trajec: %s", result.toString().c_str());

  //ROS_INFO("Exiting Planner::replanTrajec");
  return result;
}

const std::vector<RampTrajectory> Planner::replanTrajecs(const std::vector<RampTrajectory> trajecs, const MotionState ms_start) {
  //ROS_INFO("In Planner::replanTrajecs");
  std::vector<RampTrajectory> result;

  for(uint8_t i=0;i<trajecs.size();i++) {
    //ROS_INFO("i: %i trajecs.size(): %i", (int)i, (int)trajecs.size());
    RampTrajectory temp = replanTrajec(trajecs.at(i), ms_start);
    result.push_back(temp);
  }

  //ROS_INFO("Exiting Planner::replanTrajecs");
  return result;
}


/** This method will return a vector of trajectoies for the vector of paths */
const std::vector<RampTrajectory> Planner::getTrajectories(const std::vector<Path> p) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<p.size();i++) {
    //ROS_INFO("i: %i p.size(): %i", (int)i, (int)p.size());
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(p.at(i));
    result.push_back(temp);
  } // end for

  //ROS_INFO("Exiting getTrajectories");
  return result;
} // End getTrajectories





/** This method will return a vector of trajectoies for the vector of paths */
// TODO: trajectoryrequest reference?
const std::vector<RampTrajectory> Planner::getTrajectories(std::vector<ramp_msgs::TrajectoryRequest> tr) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<tr.size();i++) {
    
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(tr.at(i));
    result.push_back(temp);
  } // end for

  return result;
} // End getTrajectories




/** 
 * This function generates the initial population of trajectories,
 *  sets the paths in the Modifier class
 *  and evaluates the population
 **/
void Planner::initPopulation() 
{ 
  population_ = getPopulation(latestUpdate_, goal_, true);
  for(uint8_t i=0;i<population_.size();i++)
  {
    RampTrajectory temp = population_.get(i);
    temp.msg_.t_start = ros::Duration(0);
    population_.replace(i, temp);
  }
} // End init_population




const bool Planner::checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to) const {
  //ROS_INFO("In Planner::CheckIfSwitchCurveNecessary");
  // TODO: lastestUpdate or from's first theta?
  double thetaToSwitch, thetaCurrent = latestUpdate_.msg_.positions.at(2);

  ////ROS_INFO("to.msg.trajectory.points.size(): %i", (int)to.msg_.trajectory.points.size());
  ////ROS_INFO("to.msg.i_knotPoints.size(): %i", (int)to.msg_.i_knotPoints.size());
  //if(to.msg_.i_knotPoints.size() > 1) {
    ////ROS_INFO("to.msg.i_knotPoints.at(1): %i", (int)to.msg_.i_knotPoints.at(1));
  //}

  int kp = 1; 
  // Check if 1st two positions in "to" trajectory are the same
  // If they are, the 2nd kp is the end of a rotation
  // Theta after rotating will tell us the theta needed to move on "to" trajectory
  if( to.msg_.i_knotPoints.size() > 1 && fabs(utility_.positionDistance( to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(0)).positions,
                                            to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(kp)).positions)) < 0.0001)
  {
    ////ROS_INFO("In if positions are the same");
    thetaToSwitch = to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1) ).positions.at(2);
  }
  else if( to.msg_.i_knotPoints.size() > 1) {
    ////ROS_INFO("In else positions are not the same");
    thetaToSwitch = utility_.findAngleFromAToB(
                          to.msg_.trajectory.points.at(0), 
                          to.msg_.trajectory.points.at(
                            to.msg_.i_knotPoints.at(kp)) ); 
  }
  else 
  {
    //ROS_INFO("to trajec: %s", to.toString().c_str());
    thetaToSwitch = to.msg_.trajectory.points.at(0).positions.at(2);
  }


  //ROS_INFO("thetaCurrent: %f thetaToSwitch: %f", thetaCurrent, thetaToSwitch);
  //ROS_INFO("fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)): %f", fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)));


  // If a difference of 1 degree, compute a curve
  if(fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)) > 0.017) {
    ////ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning true");
    return true;
  }

  //ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning false");
  return false;
}



/*
 * Difference between computeFullSwitch and switchTrajectory is the result contains the evaluation result
 */
const RampTrajectory Planner::computeFullSwitch(const RampTrajectory from, const RampTrajectory to) 
{
  //ROS_INFO("In Planner::computeFullSwitch");
  RampTrajectory result;

  //ROS_INFO("to: %s", to.toString().c_str());

  // Get transition trajectory
  ros::Time tt = ros::Time::now();
  std::vector<RampTrajectory> trajecs = switchTrajectory(from, to);
  ////ROS_INFO("Time spent getting switch trajectory: %f", (ros::Time::now()-tt).toSec());

  // If a switch was possible
  if(trajecs.size() > 0)
  {
    RampTrajectory T_new = trajecs.at(1);
    Path p = T_new.path_;
    ////ROS_INFO("Before eval, T_new.path: %s", T_new.path_.toString().c_str());

    // Evaluate T_new
    ramp_msgs::EvaluationRequest er = buildEvaluationRequest(T_new);
    T_new                           = requestEvaluation(er);

    // Set misc members
    T_new.transitionTraj_   = trajecs.at(0).msg_;
    T_new.path_             = p;

    // Set result
    result                  = T_new;
    result.transitionTraj_  = trajecs.at(0).msg_;
    ////ROS_INFO("result.transitionTraj.size(): %i", (int)result.transitionTraj_.trajectory.points.size());

    ////ROS_INFO("After eval, T_new.path: %s", T_new.path_.toString().c_str());
  }

  // If a switch was not possible, just return
  // the holonomic trajectory
  else
  {
    ////ROS_WARN("A switch was not possible, returning \"to\" trajectory: %s", to.toString().c_str());
    return to;
  }


  //ROS_INFO("Exiting Planner::computeFullSwitch");
  return result;
} // End computeFullSwitch



/**
 * \brief Returns true if curve is possible from a to b at time t
 * \param t Seconds in future
 */
bool Planner::predictTransition(const RampTrajectory from, const RampTrajectory to, const double t)
{
  //ROS_INFO("In Planner::predictTransition");

  if(to.msg_.trajectory.points.size() == 0)
  {
    ////ROS_WARN("to.msg_.trajectory.points.size() == 0");
    ////ROS_WARN("Returning false");
    return false;
  }

  // Get the first two control points for the transition curve
  MotionState ms_startTrans = from.getPointAtTime(t);
  MotionState ms_endOfMovingOn = to.msg_.trajectory.points.size() > 0 ? 
    to.msg_.trajectory.points.at(0) : 
    from.msg_.trajectory.points.at(from.msg_.trajectory.points.size()-1);
  //ROS_INFO("ms_startTrans: %s", ms_startTrans.toString().c_str());
  //ROS_INFO("ms_endOfMovingOn: %s", ms_endOfMovingOn.toString().c_str());

 
  /*
   * If the robot does not have correct orientation to move on the first segment
   * then a curve cannot be planned.
   * return a blank trajectory
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    ////ROS_WARN("Cannot plan a transition curve!");
    ////ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    return false;
  }


  // Add points to segments
  std::vector<MotionState> segmentPoints;
  segmentPoints.push_back(ms_startTrans);
  segmentPoints.push_back(ms_endOfMovingOn);

  /*
   * Get 3rd control point
   * 2nd knot point should be the initial point on that trajectory's bezier 
   * Using start of Bezier rather than segment endpoint ensures that
   * the trajectory will end at the start of the Bezier
   */
  int i_goal = 1;
  if(to.msg_.i_knotPoints.size() == 1) 
  {
    i_goal = 0;
  }

  // Removed this section because we changed from getPath() to the actual path_ member
  // Else if there's self-rotation at the beginning
  else if(to.msg_.i_knotPoints.size() > 2 && 
      utility_.positionDistance(  to.path_.start_.motionState_.msg_.positions, 
                                  to.msg_.trajectory.points.at(
                                    to.msg_.i_knotPoints.at(1)).positions ) 
                                < 0.1)
  {
    i_goal = 2;
  }
  //ROS_INFO("i_goal: %i to.msg_.trajectory.points.size(): %i to.msg_.i_knotPoints.size(): %i", i_goal, (int)to.msg_.trajectory.points.size(), (int)to.msg_.i_knotPoints.size());
 
  // Set third segment point
  MotionState g(to.msg_.trajectory.points.at(to.msg_.i_knotPoints.at(i_goal)));
  segmentPoints.push_back(g);
  //ROS_INFO("After getting segment points");

  /*
   * Check misc things like same orientation, duplicate knot points, speed too fast
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    ////ROS_WARN("Cannot plan a transition curve!");
    ////ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    return false;
  }

  // Check duplicates and speeds of segment points
  for(int i=0;i<segmentPoints.size()-1;i++)
  {
    MotionState a = segmentPoints.at(i);
    MotionState b = segmentPoints.at(i+1);

    // Check duplicate
    if(utility_.positionDistance(a.msg_.positions, b.msg_.positions) < 0.1)
    {
      ////ROS_WARN("Cannot plan a transition curve because there are duplicate segment points");
      ////ROS_WARN("%s\n%s", a.toString().c_str(), b.toString().c_str());
      return false;
    }

    if( a.msg_.velocities.size() > 0 &&
        fabs(a.msg_.velocities.at(0)) > 0.0001 &&
        fabs(a.msg_.velocities.at(1)) > 0.0001)
    {
      // Check speed
      double delta_x = fabs(segmentPoints.at(i+1).msg_.positions.at(0) - 
        segmentPoints.at(i).msg_.positions.at(0));
      double delta_y = fabs(segmentPoints.at(i+1).msg_.positions.at(1) - 
        segmentPoints.at(i).msg_.positions.at(1));

      double max_gain_x = fabs(segmentPoints.at(i).msg_.velocities.at(0)) * 0.1;
      double max_gain_y = fabs(segmentPoints.at(i).msg_.velocities.at(1)) * 0.1;

      if( (max_gain_x > delta_x) ||
          (max_gain_y > delta_y) )
      {
        ROS_ERROR("The speed of the first knot point is too high to not overshoot next knot point, setting res.error=1");
        return false;
      }
    }
  } // end for


  //ROS_INFO("Done checking segments");
  

  
  /*
   * After getting both segments, check if they have the same orientation
   * If so, just return the rest of movingOn, no need to plan a transition trajectory
   */
  double thetaS1 = utility_.findAngleFromAToB(segmentPoints.at(0).msg_.positions, 
                                              segmentPoints.at(1).msg_.positions);
  double thetaS2 = utility_.findAngleFromAToB(segmentPoints.at(1).msg_.positions, 
                                              segmentPoints.at(2).msg_.positions);
  //ROS_INFO("Theta 1: %f Theta 2: %f", thetaS1, thetaS2);
  if( fabs(utility_.findDistanceBetweenAngles(thetaS1, thetaS2)) < 0.13 )
  {
    ////ROS_WARN("Segments have the same orientation - no need to plan a transition curve, use a straight-line trajectory");
    return true;
  }


  /*
   * See if a curve can be planned
   */
  // 0.1 = lambda
  BezierCurve curve;
  for(float lambda=0.1;lambda < 0.85;lambda+=0.1f)
  {
    curve.init(segmentPoints, lambda, ms_startTrans);
    if(curve.verify())
    {
      return true;
    }
  }
  
  ////ROS_INFO("Curve formed for prediction: %s", utility_.toString(curve.getMsg()).c_str());

  //ROS_INFO("Exiting Planner::predictTransition");
  return false;
}


/*
 * Returns a vector of trajectories
 * Index 0 = transition trajectory
 * Index 1 = Full switching trajectory
 */
const std::vector<RampTrajectory> Planner::switchTrajectory(const RampTrajectory from, const RampTrajectory to) 
{
  //ROS_INFO("In Planner::switchTrajectory");
  //ROS_INFO("from: %s\nto.path: %s", from.toString().c_str(), to.path_.toString().c_str());
  std::vector<RampTrajectory> result;

  /*
   * Find the best planning cycle to switch 
   */
  //ROS_INFO("generationsPerCC_: %i c_pc+1: %i", generationsPerCC_, (c_pc_+1));
  uint8_t pc = generationsPerCC_+1;
  for(int i_pc=generationsPerCC_-1; i_pc > (c_pc_+1); i_pc--)
  {
    //ROS_INFO("Look for transition at i_pc: %i", i_pc);

    double t_pc = i_pc * planningCycle_.toSec();

    // Get t and the motion state along moving at time t
    /*double t = errorReduction_                      ? 
                from.getT() - (t_fixed_cc_ - t_pc)  : 
                t_pc ;*/
    ////ROS_INFO("t_pc: %f t: %f", t_pc, t);
    MotionState ms = from.getPointAtTime(t_pc);

    if(predictTransition(from, to, t_pc))
    {
      pc = i_pc;
      break;
    }
    /*else
    {
      ROS_INFO("Prediction returns false");
    }*/
  } // end for
 

  /*
   * Call getTransitionTrajectory
   * if we can find one before next CC
   */
  if(pc  < generationsPerCC_)
  {
    double t_pc = pc * planningCycle_.toSec();

    // Get t and the motion state along moving at time t
    double t = errorReduction_                      ? 
                from.getT() - (t_fixed_cc_ - t_pc)  : 
                t_pc ;
    //ROS_INFO("pc: %i t_pc: %f t: %f", pc, t_pc, t);

    ros::Time t_start_trans = ros::Time::now();
    RampTrajectory switching  = getTransitionTrajectory(from, to, t);
    ////ROS_INFO("Time spent getting transition: %f", (ros::Time::now()-t_start_trans).toSec());
    RampTrajectory full       = switching.clone();
    //ROS_INFO("Switching trajectory: %s", switching.toString().c_str());

    // If robot is at goal, full should only be 1 point,
    // check for this to prevent crashing
    if(full.msg_.i_knotPoints.size() > 1)
    {
      // Keep a counter for the knot points
      // Start at 1 because that should be the starting knot point of the curve
      int c_kp = 1;
      
      // Check if there's rotation at the beginning, if so increment c_kp
      // TODO: Better way of doing this
      // Used to be else-if
      if(utility_.positionDistance( to.msg_.trajectory.points.at(0).positions,
            to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1)).positions ) < 0.1)
      {
        //std::cout<<"\nIncrementing c_kp";
        c_kp++;
      }
      //ROS_INFO("c_kp: %i", c_kp);
      //ROS_INFO("c_kp: %i i_knotPoints.size(): %i", c_kp, (int)to.msg_.i_knotPoints.size());

      // Set full as the concatenating of switching and to
      full        = switching.concatenate(to, c_kp);

      ////ROS_INFO("full.path: %s", full.path_.toString().c_str());
      ////ROS_INFO("full.path after: %s", full.path_.toString().c_str());

      // Set the proper ID, path, and t_starts
      full.msg_.id            = to.msg_.id;
      full.path_              = to.path_;

      //double T = movingOn_.msg_.trajectory.points.at(movingOn_.msg_.trajectory.points.size()-1).time_from_start.toSec();
      //full.msg_.t_start       = c_pc_ > 0 ? ros::Duration(t_fixed_cc_ - (T - t)) : ros::Duration(t);
      full.msg_.t_start       = ros::Duration(pc * planningCycle_.toSec());
      switching.msg_.t_start  = full.msg_.t_start;

      full.transitionTraj_    = switching.msg_;

      result.push_back(switching);
      result.push_back(full);
    } // end if size > 1
  } // end if switch possible
  /*else
  {
    ROS_INFO("pc: %i generationsPerCC_: %i", pc, generationsPerCC_);
  }*/
 

  //ROS_INFO("Exiting Planner::switchTrajectory");
  return result;
} // End switchTrajectory





const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory trj_movingOn, 
    const RampTrajectory trj_target, const double t) 
{
  /*ROS_INFO("In Planner::getTransitionTrajectory");
  ROS_INFO("t: %f", t);
  ROS_INFO("trj_movingOn: %s", trj_movingOn.toString().c_str());
  ROS_INFO("trj_target: %s", trj_target.toString().c_str());*/

  /* 
   * The segment points for a transition are
   * 1) Motion state at time t along trj_movingOn
   * 2) Motion state at the end of trj_movingOn
   * 3) Motion state of the target's first knot point (normally start of Bezier curve)
   * The robot must be oriented towards CP 2 at CP 1 in order for a curve to be possible
   * If the robot is not oriented this way, log a warning and return a blank trajectory
   */
 
  // Get the first two control points for the transition curve
  MotionState ms_startTrans = trj_movingOn.getPointAtTime(t);
  MotionState ms_endOfMovingOn = trj_target.msg_.trajectory.points.size() > 0 ? 
    trj_target.msg_.trajectory.points.at(0) : 
    trj_movingOn.msg_.trajectory.points.at(trj_movingOn.msg_.trajectory.points.size()-1);
  ////ROS_INFO("ms_startTrans: %s", ms_startTrans.toString().c_str());
  ////ROS_INFO("ms_endOfMovingOn: %s", ms_endOfMovingOn.toString().c_str());

 
  /*
   * If the robot does not have correct orientation to move on the first segment
   * then a curve cannot be planned.
   * return a blank trajectory
   */
  if(fabs(utility_.findDistanceBetweenAngles( 
        ms_startTrans.msg_.positions.at(2), ms_endOfMovingOn.msg_.positions.at(2))) > 0.12 ) 
  {
    ////ROS_WARN("Cannot plan a transition curve!");
    ////ROS_WARN("Robot does not have correct orientation to move on first segment of a transition curve");
    ////ROS_WARN("startTrans: %s\nendOfMovingOn: %s", ms_startTrans.toString().c_str(), ms_endOfMovingOn.toString().c_str());
    RampTrajectory blank;
    return blank;
  }


  // Add points to segments
  std::vector<MotionState> segmentPoints;
  segmentPoints.push_back(ms_startTrans);
  segmentPoints.push_back(ms_endOfMovingOn);



  /*
   * Get 3rd control point
   * 2nd knot point should be the initial point on that trajectory's bezier 
   * Using start of Bezier rather than segment endpoint ensures that
   * the trajectory will end at the start of the Bezier
   */
  int i_goal = 1;
  if(trj_target.msg_.i_knotPoints.size() == 1) 
  {
    i_goal = 0;
  }

  // Removed this section because we changed from getPath() to the actual path_ member
  // Else if there's self-rotation at the beginning
  else if(trj_target.msg_.i_knotPoints.size() > 2 && 
      utility_.positionDistance(  trj_target.path_.start_.motionState_.msg_.positions, 
                                  trj_target.msg_.trajectory.points.at(
                                    trj_target.msg_.i_knotPoints.at(1)).positions ) 
                                < 0.1)
  {
    i_goal = 2;
  }
  //ROS_INFO("i_goal: %i", i_goal);
 
  MotionState g(trj_target.msg_.trajectory.points.at(trj_target.msg_.i_knotPoints.at(i_goal)));
  segmentPoints.push_back(g);

  /*//ROS_INFO("Segment points:");
  for(int i=0;i<segmentPoints.size();i++)
  {
    //ROS_INFO("Segment point [%i]: %s", i, segmentPoints.at(i).toString().c_str());
  }*/

  // Make the path of the transition curve
  Path p(segmentPoints);

  /*
   * After getting both segments, check if they have the same orientation
   * If so, just return the rest of movingOn, no need to plan a transition trajectory
   */
  double thetaS1 = utility_.findAngleFromAToB(segmentPoints.at(0).msg_.positions, 
                                              segmentPoints.at(1).msg_.positions);
  double thetaS2 = utility_.findAngleFromAToB(segmentPoints.at(1).msg_.positions, 
                                              segmentPoints.at(2).msg_.positions);
  ////ROS_INFO("Theta 1: %f Theta 2: %f", thetaS1, thetaS2);
  if( fabs(utility_.findDistanceBetweenAngles(thetaS1, thetaS2)) < 0.13 )
  {
    ////ROS_WARN("Segments have the same orientation - no need to plan a transition curve, use a straight-line trajectory");
    ////ROS_WARN("Removing the following point at index 1 of the Path: %s", p.at(1).toString().c_str());
    p.all_.erase(p.all_.begin()+1);
  }
  /*else
  {
    //ROS_INFO("Segments orientation difference: %f", 
        utility_.findDistanceBetweenAngles(thetaS1, thetaS2));
  }*/


  /*
   * Get curve
   */

  // Build request and get trajectory
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.request.type = TRANSITION;

  RampTrajectory trj_transition = requestTrajectory(tr);

  ////ROS_INFO("trj_transition: %s", trj_transition.toString().c_str());
  //ROS_INFO("Exiting Planner::getTransitionTrajectory");
  return trj_transition;
} // End getTransitionTrajectory






/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
// Not const because it calls getIRT() to get an index for the trajectory if an id is not passed in
const RampTrajectory Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr, const int id) 
{
  RampTrajectory result;
  //std::cout<<"\nid: "<<id;

  
  ros::Time t_start = ros::Time::now();
  if(h_traj_req_->request(tr)) 
  {
    trajec_durs_.push_back(ros::Time::now() - t_start);
    
    // Set the actual trajectory msg
    result.msg_         = tr.response.trajectory;

    // Set things the traj_gen does not have
    result.msg_.t_start = ros::Duration(t_fixed_cc_);

    // Set the paths (straight-line and bezier)
    result.path_        = tr.request.path;

    // *** Set the previous knot point
    //result.ms_prevSP_ = tr.request.path.points.at(0).motionState;

    // Set the ID of the trajectory
    if(id != -1) {
      result.msg_.id = id;
    }
    else {
      result.msg_.id = getIRT();
    }
  }
  else 
  {
    ROS_ERROR("An error occurred when requesting a trajectory");
  }

  //ROS_INFO("Exiting Planner::requestTrajectory, t_start: %f", result.msg_.t_start.toSec());
  return result;
}



const RampTrajectory Planner::requestTrajectory(const Path p, const int id) {
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  RampTrajectory result = requestTrajectory(tr, id);
  //ROS_INFO("Exiting Planner::requestTrajectory");
  return result;
}



/** Request an evaluation */
const RampTrajectory Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) {
  //ROS_INFO("In Planner::requestEvaluation");
  RampTrajectory result = er.request.trajectory; 
  //ROS_INFO("result.t_start: %f", result.msg_.t_start.toSec());
  
  if(h_eval_req_->request(er)) 
  {
    result.msg_.fitness           = er.response.fitness;
    result.msg_.feasible          = er.response.feasible;
    result.msg_.t_firstCollision  = er.response.t_firstCollision;
  }
  else {
    ROS_ERROR("An error occurred when evaluating a trajectory");
  }
  
  //ROS_INFO("Exiting Planner::requestEvaluation");
  return result;
}


const RampTrajectory Planner::requestEvaluation(const RampTrajectory traj) {
  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(traj);
  RampTrajectory result           = requestEvaluation(er);

  // Set non-evaluation related members
  result.path_                = traj.path_;
  //result.bezierPath_          = traj.bezierPath_;
  result.msg_.i_subPopulation = traj.msg_.i_subPopulation; 

  return result;
} 

/******************************************************
 ****************** Modifying Methods *****************
 ******************************************************/


/** Modify a Path */
const std::vector<Path> Planner::modifyPath() 
{ 
  //ROS_INFO("About to modify a path, pop is: %s\n%s", population_.get(0).toString().c_str(), population_.get(1).toString().c_str());
  return modifier_->perform(population_);
  //return modifier_->perform(population_at_cc_);
}



/** Modify a trajectory */ 
const std::vector<RampTrajectory> Planner::modifyTrajec() 
{
  //ROS_INFO("In Planner::modifyTrajec");
  std::vector<RampTrajectory> result;
  

  // The process begins by modifying one or more paths
  std::vector<Path> modded_paths = modifyPath();
  ////ROS_INFO("Number of modified paths: %i", (int)modded_paths.size());


  // For each targeted path,
  for(unsigned int i=0;i<modded_paths.size();i++) {
    //std::cout<<"\nramp_planner: Modifying trajectory "<<(int)i;
    
    // Get trajectory
    RampTrajectory temp = requestTrajectory(modded_paths.at(i));
    result.push_back(temp);
  
  } // end for
  
  //ROS_INFO("Exiting Planner::modifyTrajec");
  return result;
} // End modifyTrajectory







/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  set the new best trajectory,
 *  and return the index of the new best trajectory */
const ModificationResult Planner::modification() 
{
  //ROS_INFO("In Planner::modification()");
  ModificationResult result;

  // Modify 1 or more trajectories
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  //ROS_INFO("Modification trajectories obtained: %i", (int)mod_trajec.size());
  
  Population popCopy = population_;
  Population trans_popCopy = transPopulation_;
  //Population popCopy = population_at_cc_;
  

  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) 
  {
    //ROS_INFO("Modified trajectory: %s", mod_trajec.at(i).toString().c_str());
    //std::cout<<"\nramp_planner: Evaluating trajectory "<<(int)i<<"\n";

    // Evaluate the new trajectory
    mod_trajec.at(i) = evaluateTrajectory(mod_trajec.at(i));


    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    // If it was successfully added, push its index onto the result
    ////ROS_INFO("Trying to add modified trajectory");
    int index=-1;
    if(cc_started_)
    {
      if(popCopy.replacementPossible(mod_trajec.at(i)))
      {
        RampTrajectory trans = computeFullSwitch(movingOn_, mod_trajec.at(i));
        index = trans_popCopy.add(trans);
        if(index > -1)
        {
          popCopy.replace(index, mod_trajec.at(i));
          result.i_modified_.push_back(index);
        }
      } // end if replacementPossible
    } // end if cc_started
    else
    {
      index = popCopy.add(mod_trajec.at(i));
      if(index > -1)
      {
        result.i_modified_.push_back(index);
      } // end if added
    } // end else
    
    //ROS_INFO("Added at index %i", index);
    /*if(index > -1) 
    {
      if(!cc_started_)
      {
        population_.replace(index, mod_trajec.at(i));
        result.i_modified_.push_back(index);
      }
      else
      {
        //ROS_INFO("Computing switch to modified trajectory");
        RampTrajectory trans = computeFullSwitch(movingOn_, mod_trajec.at(i));
        bool index_trans = trans_popCopy.canReplace(trans, index);
        transPopulation_.replace(index, trans);
      }
    }
    else 
    {
      //ROS_INFO("Modification Trajectory not added to population");
    }*/

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) 
    {
      popCopy.createSubPopulations();
      //trans_popCopy.createSubPopulations();
    }
  } // end for


  result.popNew_    = popCopy;
  result.transNew_  = trans_popCopy;
  ////ROS_INFO("After modification, pop now: %s", result.popNew_.toString().c_str());

  //ROS_INFO("Exiting Planner::modification");
  return result;
} // End modification






void Planner::stopForDebugging() {

  h_parameters_.setImminentCollision(true); 

  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();
}

void Planner::restartAfterDebugging() {
  h_parameters_.setImminentCollision(false); 

  controlCycleTimer_.start();
  planningCycleTimer_.start();
  imminentCollisionTimer_.start();
}

void Planner::pause() {
  stopForDebugging();
  std::cout<<"\nPress Enter to continue\n";
  std::cin.get();
  restartAfterDebugging();
}




/** 
 * This method will replace the starting motion state of each path
 * with s and will update the modifier's paths 
 * */
void Planner::updatePathsStart(const MotionState s) 
{
  //ROS_INFO("In Planner::updatePathsStart");

  for(unsigned int i=0;i<population_.paths_.size();i++) {
    population_.paths_.at(i).start_ = s;

    population_.paths_.at(i).all_.erase (population_.paths_.at(i).all_.begin());
    population_.paths_.at(i).all_.insert(population_.paths_.at(i).all_.begin(), s);
  }

  //ROS_INFO("Exiting Planner::updatePathsStart");
} // End updatePathsStart


const RampTrajectory Planner::offsetTrajectory(const RampTrajectory t, const MotionState diff) const
{
  RampTrajectory result = t;  

  result.offsetPositions(diff);

  return result;
}

/*
 * diff will be the amount to offset. Only the positions will be offset
 */
const Population Planner::offsetPopulation(const Population pop, const MotionState diff) const
{
  //ROS_INFO("In Planner::offsetPopulation");
  Population result = pop;

  //ROS_INFO("diff: %s", diff.toString().c_str());
 
   
  for(uint8_t i=0;i<pop.size();i++)
  {
    //ROS_INFO("Trajectory %i", i);
    
    RampTrajectory temp = pop.get(i);
    temp.offsetPositions(diff);
    
    result.replace(i, temp);
  }
   
  //ROS_INFO("Exiting Planner::offsetPopulation");
  return result;
}




const MotionState Planner::errorCorrection()  
{
  //ROS_INFO("In Planner::errorCorrection");
  MotionState result;

  //ROS_INFO("c_pc: %i", (int)c_pc_);
  //ROS_INFO("m_i.size(): %i", (int)m_i_.size());
 
  //ROS_INFO("m_i[%i]: %s", c_pc_, m_i_.at(c_pc_).toString().c_str());
  //ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Get the difference between robot's state and what state it should be at
  MotionState diff = m_i_.at(c_pc_).subtractPosition(latestUpdate_, true);
  error_correct_val_pos_.push_back( sqrt( pow(diff.msg_.positions.at(0), 2) + pow(diff.msg_.positions.at(1),2) ) );
  error_correct_val_or_.push_back(diff.msg_.positions.at(2));
 
  //ROS_INFO("m_cc: %s\ndiff: %s", m_cc_.toString().c_str(), diff.toString().c_str());

  // subtractPosition that difference from startPlanning
  result = m_cc_.subtractPosition(diff, true);

  // Set new theta
  result.msg_.positions.at(2) = latestUpdate_.msg_.positions.at(2);


  //ROS_INFO("Exiting Planner::errorCorrection");
  return result;
}




void Planner::planningCycleCallback() {
  ros::Time t_start = ros::Time::now();
  ROS_INFO("*************************************************");
  ROS_INFO("Planning cycle occurring, generation %i", generation_);
  ROS_INFO("*************************************************");
 
  ROS_INFO("Time since last CC: %f", (ros::Time::now()-t_prevCC_).toSec());


 
  MotionState diff;

  bool did_error_correction = false;

  // Make sure not too many PC occur before next CC
  //if(!cc_started_ || c_pc_ < generationsPerCC_) 

  /*
   * Error correction
   */
  // Must have started control cycles
  // c_pc < total number of PC's per CC
  // errorReduction is true
  // Driving straight
  /*if(cc_started_ && c_pc_ < generationsPerCC_ &&
      errorReduction_ && fabs(latestUpdate_.msg_.velocities.at(2)) < 0.1)
  {
    ros::Time t_start_error = ros::Time::now();

    // TODO: Is this if statement needed? Isn't it essentially checking that we are not
    // moving on a curve, which is what the previous one checks?
    // If not first PC and best trajectory has no curve
    // best curve has a curve and we aren't moving on it
    if(population_.getBest().msg_.curves.size() == 0    || 
        ( population_.getBest().msg_.curves.size() > 0  && 
          population_.getBest().msg_.curves.at(0).u_0 < 0.0001))
    {
        //ROS_INFO("Doing error correction");
        //ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
        // Do error correction
        diff = m_i_.at(c_pc_).subtractPosition(latestUpdate_, true);
        startPlanning_ = errorCorrection();
        //ROS_INFO("diff: %s", diff.toString().c_str());

        ////ROS_INFO("Updating movingOn");
        Path p(latestUpdate_, startPlanning_);
        movingOn_ = movingOnCC_;
        movingOn_.offsetPositions(diff);
        
        //ROS_INFO("Corrected startPlanning_: %s", startPlanning_.toString().c_str());
        //ROS_INFO("Corrected movingOn_: %s", movingOn_.toString().c_str());

        //ROS_INFO("Before offset, pop: %s\n%stransPop: %s\n%s", 
            //population_.get(0).toString().c_str(),
            //population_.get(1).toString().c_str(),
            //transPopulation_.get(0).toString().c_str(),
            //transPopulation_.get(1).toString().c_str());

        population_       = offsetPopulation(population_at_cc_, diff);
        transPopulation_  = offsetPopulation(transPopulation_at_cc_, diff);

        population_       = evaluatePopulation(population_);
        transPopulation_  = evaluatePopulation(transPopulation_);

        //ROS_INFO("After doing error correction, new pop: %s\n%s\n\n\nnew trans pop: %s\n%s", 
            //population_.get(0).toString().c_str(),
            //population_.get(1).toString().c_str(),
            //transPopulation_.get(0).toString().c_str(),
            //transPopulation_.get(1).toString().c_str());

        did_error_correction = true;
    } // end if doing error correction
    else
    {
      //ROS_INFO("Not doing error correction");
      //ROS_INFO("c_pc: %i population_.getBest().msg_.curves.size(): %i", c_pc_, (int)population_.getBest().msg_.curves.size());
      //ROS_INFO("population_.getBest().msg_.curves.at(0).u_0: %f", population_.getBest().msg_.curves.at(0).u_0); 
    }

    error_correct_durs_.push_back(ros::Time::now() - t_start_error);
  } // end if doing error correction */
  /*else
  {
    //ROS_INFO("Not doing error correction");
    //ROS_INFO("cc_started_: %s c_pc_: %i errorReduction_: %s fabs(latestUpdate_.msg_.velocities.at(2)): %f", 
        cc_started_ ? "True" : "False", c_pc_,
        errorReduction_ ? "True" : "False", 
        fabs(latestUpdate_.msg_.velocities.at(2)));
  }*/



  /*
   * Modification
   */
  if(modifications_) 
  {
    ROS_INFO("*****************************");
    ROS_INFO("Performing modification");
    ros::Time t = ros::Time::now();
    ModificationResult mod = modification();
    mutate_durs_.push_back(ros::Time::now() - t);
    ROS_INFO("Done with modification");
    ROS_INFO("*****************************");


    // cc_started needed? still want to replace if they haven't started, only need cc_started when switching 
    // TODO: cc_started used to be a predicate here
    if(mod.i_modified_.size() > 0 &&
        !population_.get(0).path_.at(0).motionState_.equals(goal_))
    {
      //ROS_INFO("In if trajectory added");
      population_       = mod.popNew_;
      transPopulation_  = mod.transNew_;
      //population_at_cc_ = mod.popNew_;
      //transPopulation_at_cc_ = mod.transNew_;
      
      ////ROS_INFO("Modification changed population");
      /*for(int i=0;i<mod.i_modified_.size();i++)
      {
        RampTrajectory temp = mod.popNew_.get(mod.i_modified_.at(i));
        mod.popNew_.replace(mod.i_modified_.at(i), temp);

        temp = mod.transNew_.get(mod.i_modified_.at(i));
        mod.transNew_.replace(mod.i_modified_.at(i), temp);
      }*/
      //population_       = mod.popNew_;
      //transPopulation_  = mod.transNew_;
      // Pop = popnew was outside this block - why?
      //ROS_INFO("New pop: %s", population_.toString().c_str());
      //ROS_INFO("New transPop: %s", transPopulation_.toString().c_str());


      controlCycle_ = transPopulation_.getBest().msg_.t_start;
      controlCycleTimer_.setPeriod(transPopulation_.getBest().msg_.t_start, false);
      //ROS_INFO("Modification: new CC timer: %f", transPopulation_.getBest().msg_.t_start.toSec());
    } // end if trajectory added
    /*else
    {
      ROS_INFO("No trajectory added");
    }*/
  } // end if modifications



  /* 
   * Finish up
   */
  // t=t+1
  generation_++;
  c_pc_++;

  if(cc_started_)
  {
    sendPopulation(transPopulation_);
  }
  else
  {
    sendPopulation(population_);
  }
  //ROS_INFO("population.bestID: %i", population_.calcBestIndex());

  //ROS_INFO("Pop: %s", population_.toString().c_str());
  //ROS_INFO("Exiting PC at time: %f", ros::Time::now().toSec());
  ROS_INFO("Time spent in PC: %f", (ros::Time::now() - t_start).toSec());
  pc_durs_.push_back(ros::Time::now() - t_start);
  //ROS_INFO("********************************************************************");
  //ROS_INFO("Generation %i completed", (generation_-1));
  //ROS_INFO("********************************************************************");
} // End planningCycleCallback




const uint8_t Planner::computeSwitchPC(const RampTrajectory target, const RampTrajectory moving)
{
  //ROS_INFO("In Planner::computeSwitchPC(RampTrajectory, RampTrajectory)");
  // Worst case - no curve possible, stop and rotate at next CC
  int result = generationsPerCC_;
  //ROS_INFO("moving: %s", moving.toString().c_str());

  for(int i_pc=result-1;i_pc>0;i_pc--)
  {
    //ROS_INFO("i_pc: %i", i_pc);

    // Get t and the motion state along moving at time t
    double t = i_pc*planningCycle_.toSec();
    MotionState ms = moving.getPointAtTime(t);

    // Must be moving straight
    // TODO: Change this to oriented towards end of moving
    if(fabs(ms.msg_.velocities.at(2)) < 0.001)
    {
      RampTrajectory trans = getTransitionTrajectory(moving, target, t);

      //ROS_INFO("trans: %s", trans.toString().c_str());

      // If we are able to plan a curve and it's not the best
      if( (trans.msg_.i_knotPoints.size() == 2) ||
          (trans.msg_.curves.size() > 0 &&
          trans.msg_.i_knotPoints.size() < 5) )
      {
        //ROS_INFO("Able to plan a transition curve!");
      } // end if
      else
      {
        //ROS_INFO("Unable to plan a transition curve!");
        //ROS_INFO("trans.msg_.curves.size(): %i", (int)trans.msg_.curves.size());
        //ROS_INFO("trans.msg_.i_knotPoints.size(): %i", (int)trans.msg_.i_knotPoints.size());
      }
    } // end if moving on straight line
    else
    {
      //ROS_INFO("Stopping! Not moving on straight line at pc: %i", i_pc);
      i_pc = 0;
    }
  }

  //ROS_INFO("Exiting Planner::computeSwitchPC(RampTrajectory, RampTrajectory)");
  return result;
} // End computeSwitchPC



const uint8_t Planner::computeSwitchPC(const Population pop, const RampTrajectory moving) 
{
  //ROS_INFO("In Planner::computeSwitchPC()");

  int result = generationsPerCC_;
  ////ROS_INFO("moving: %s", moving.toString().c_str());

  for(int i_pc=result-1;i_pc>0;i_pc--)
  {
    ////ROS_INFO("i_pc: %i", i_pc);
    double t = i_pc*planningCycle_.toSec();

    // If the robot is moving on a straight line at this pc, compute curve
    MotionState ms = moving.getPointAtTime(t);
    ////ROS_INFO("ms at i_pc %i: %s", i_pc, ms.toString().c_str());
    if(fabs(ms.msg_.velocities.at(2)) < 0.001)
    {

      // Go through each trajectory and try to get a transition curve
      for(uint8_t i=0;i<pop.size();i++)
      {
        ////ROS_INFO("Inner for, i: %i", i);
        RampTrajectory T_new = getTransitionTrajectory(moving, pop.get(i), t);
        ////ROS_INFO("T_new: %s", T_new.toString().c_str());

        // If we are able to plan a curve
        if( (T_new.msg_.i_knotPoints.size() == 2) ||
            ( T_new.msg_.curves.size() > 0 &&
              T_new.msg_.i_knotPoints.size() < 5) )
        {
          //ROS_INFO("Able to plan a transition curve!");
        } // end if
        else
        {
          //ROS_INFO("Unable to plan a transition curve!");
          //ROS_INFO("T_new.msg_.curves.size(): %i", (int)T_new.msg_.curves.size());
          //ROS_INFO("T_new.msg_.i_knotPoints.size(): %i", (int)T_new.msg_.i_knotPoints.size());
          i=pop.size();
        }
      } // end for
    } // end if moving on straight line
    else
    {
      //ROS_INFO("Stopping! Not moving on straight line at pc: %i", i_pc);
      i_pc = 0;
    }
  } // end for

  ////ROS_INFO("Result: %i", result);
  //ROS_INFO("Exiting Planner::computeSwitchPC");
  return result;
}



const Population Planner::getTransPop(const Population pop, const RampTrajectory movingOn)
{
  //ROS_INFO("In Planner::getTransPop");
  ////ROS_INFO("pop: %s", pop.toString().c_str());
  Population result = pop;


  // Go through the population and get:
  // 1) planning cycle to switch at
  // 2) transition trajectory
  for(uint8_t i=0;i<pop.size();i++)
  {
    //ROS_INFO("i: %i", i);
    RampTrajectory temp = computeFullSwitch(movingOn_, pop.get(i));
    result.replace(i, temp);
  }

  ////ROS_INFO("Trans pop full: %s", result.toString().c_str());

  //ROS_INFO("Exiting Planner::getTransPop");
  return result;
}







/** This methed runs the tasks needed to do a control cycle */
void Planner::doControlCycle() 
{
  //ROS_WARN("Control Cycle %i occurring at Time: %f", num_cc_, ros::Time::now().toSec());
  ROS_INFO("controlCycle_: %f", controlCycle_.toSec());
  ROS_INFO("Time between control cycles: %f", (ros::Time::now() - t_prevCC_).toSec());
  t_prevCC_ = ros::Time::now();
  ROS_INFO("Number of planning cycles that occurred between CC's: %i", c_pc_);

  ros::Time t = ros::Time::now();


  // Set the bestT
  RampTrajectory bestT = transPopulation_.getBest();

  ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  MotionState diff = bestT.path_.at(0).motionState_.subtractPosition(latestUpdate_);
  ////ROS_INFO("diff: %s", diff.toString().c_str());

  // Send the best trajectory and set movingOn
  ////ROS_INFO("Sending best");
  ROS_INFO("bestT: %s", bestT.toString().c_str());
  sendBest();
  ////ROS_INFO("After sendBest");

  
  // Set gensPerCC based on CC time
  //generationsPerCC_ = controlCycle_.toSec() / planningCycle_.toSec();
  /*//ROS_INFO("CC Time: %f PC Time: %f generationsPerCC_: %i", 
      controlCycle_.toSec(), 
      planningCycle_.toSec(),
      generationsPerCC_);*/



  ////ROS_INFO("Setting movingOn_");
  movingOnCC_             = bestT.getSubTrajectory(t_fixed_cc_);
  movingOn_               = movingOnCC_;
  movingOn_.msg_.t_start  = ros::Duration(0);
  movingOn_               = evaluateTrajectory(movingOn_);
  moving_on_coll_         = !movingOn_.msg_.feasible;
  //ROS_INFO("movingOn: %s", movingOn_.toString().c_str());

  // Reset planning cycle count
  c_pc_ = 0;

  // The motion state that we should reach by the next control cycle
  m_cc_ = bestT.getPointAtTime(t_fixed_cc_);

  // At CC, startPlanning is assumed to be perfect (no motion error accounted for yet)
  startPlanning_ = m_cc_;
  ////ROS_INFO("New startPlanning_: %s", startPlanning_.toString().c_str());

  //ROS_INFO("Before adaptation and evaluation, pop size: %i pop: %s\nDone printing pop", population_.size(), population_.toString().c_str());
  //ROS_INFO("transPop.bestID: %i", transPopulation_.calcBestIndex());

  // Adapt and evaluate population
  //ROS_INFO("About to adapt, controlCycle_: %f", controlCycle_.toSec());
  ros::Time t_startAdapt = ros::Time::now();
  population_ = adaptPopulation(transPopulation_, startPlanning_, ros::Duration(t_fixed_cc_));
  population_ = evaluatePopulation(population_);
  ros::Duration d_adapt = ros::Time::now() - t_startAdapt;
  adapt_durs_.push_back(d_adapt);
  //ROS_INFO("After adaptation and evaluation, pop size: %i pop: \n%s\nDone printing pop", population_.size(), population_.toString().c_str());
  ////ROS_INFO("Time spent adapting: %f", d_adapt.toSec());
 
  if(population_.calcBestIndex() != transPopulation_.calcBestIndex())
  {
    //ROS_WARN("Population (holo) best ID: %i\nPopulation (non-holo) best ID: %i", population_.calcBestIndex(), transPopulation_.calcBestIndex());
    ////ROS_INFO("Pop: %s", population_.toString().c_str());
    ////ROS_INFO("Trans Pop: %s", transPopulation_.toString().c_str());
  }


  //ROS_INFO("Before getTransPop: %s", transPopulation_.toString().c_str());
 
  // Find the transition (non-holonomic) population and set new control cycle time
  ros::Time t_startTrans = ros::Time::now();
  transPopulation_  = getTransPop(population_, movingOn_);
  transPopulation_  = evaluatePopulation(transPopulation_);
  controlCycle_     = ros::Duration(transPopulation_.getBest().msg_.t_start.toSec());
  ros::Duration d_trans = ros::Time::now() - t_startTrans;
  trans_durs_.push_back(d_trans);
  
  //ROS_INFO("After finding transition population, controlCycle period: %f", controlCycle_.toSec());
  /*ROS_INFO("New transPop: %s\n\n%s", 
      transPopulation_.get(0).toString().c_str(),
      transPopulation_.get(1).toString().c_str());*/
  //ROS_INFO("Time spent getting trans pop: %f", d_trans.toSec());

  population_at_cc_       = population_;
  transPopulation_at_cc_  = transPopulation_;


  // If error reduction
  // Set pop_orig_ and totalDiff to 0's
  if(errorReduction_) 
  {
    m_i_ = setMi(movingOn_);
  }
 
  // Create sub-populations if enabled
  if(subPopulations_) 
  {
    population_.createSubPopulations();
  }
 
  // Send the population to trajectory_visualization
  sendPopulation(transPopulation_);
  
  /*//ROS_INFO("Control Cycle %i Ending, next one occurring in %f seconds", 
      num_cc_, controlCycle_.toSec());*/

  ////ROS_INFO("Control Cycle: new CC timer: %f", controlCycle_.toSec());
  controlCycleTimer_.setPeriod(controlCycle_, false);

  ros::Duration d_cc = ros::Time::now() - t;
  cc_durs_.push_back(d_cc);
 
  num_cc_++;
  //ROS_INFO("Time spent in CC: %f", d_cc.toSec());
  //ROS_INFO("Exiting Planner::doControlCycle");
} // End doControlCycle





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent& e) {
  // Restart the planning cycles
  planningCycleTimer_.stop();
  planningCycleTimer_.start();
  
  ROS_INFO("*************************************************");
  ROS_INFO("  Control cycle timer event happening  ");
  ROS_INFO("  e.last_expected: %f\n  e.last_real: %f\n  current_expected: %f\n  current_real: %f\n  profile.last_duration: %f",
      e.last_expected.toSec(), e.last_real.toSec(), e.current_expected.toSec(), e.current_real.toSec(), e.profile.last_duration.toSec());
  ROS_INFO("Time since last: %f", (e.current_real - e.last_real).toSec());
  ROS_INFO("*************************************************");
  
  //ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Do the control cycle
  doControlCycle();

  //ros::WallDuration diff = controlCycle_ - e.profile.last_duration;
  
  //ros::NodeHandle h; 
  //controlCycleTimer_ = h.createWallTimer(controlCycle_, &Planner::controlCycleCallback, this)
  //e.profile.last_duration = e.profile.last_duration - diff;
  
  // Set flag showing that CCs have started
  if(!cc_started_) 
  {
    cc_started_ = true;
    h_parameters_.setCCStarted(true); 
  }
    
  //ROS_INFO("Leaving Control Cycle, period: %f", controlCycle_.toSec());
} // End controlCycleCallback











/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {
  ////ROS_INFO("Sending best trajectory: %s", population_.get(population_.calcBestIndex()).toString().c_str());

  if(!stop_) {
    RampTrajectory best = transPopulation_.getBest();
    //RampTrajectory best = bestTrajec_;

    // If infeasible and too close to obstacle, 
    // Stop the robot by sending a blank trajectory
    if(!best.msg_.feasible && (best.msg_.t_firstCollision.toSec() < 3.f)) 
    {
      std::cout<<"\nCollision within 3 seconds! Stopping robot!\n";
    }
    else if(!best.msg_.feasible) {
      //ROS_INFO("Best trajectory is not feasible! Time until collision: %f", best.msg_.t_firstCollision.toSec());
    }
    
    best.msg_.header.stamp = ros::Time::now();
    h_control_->send(best.msg_);
  } // end if not stopped
  else {
    //ROS_INFO("Sending blank!");
    RampTrajectory blank;
    h_control_->send(blank.msg_);
  }
} // End sendBest







/** Send the whole population of trajectories to the trajectory viewer */
void Planner::sendPopulation(const Population pop) const 
{
  ramp_msgs::Population msg;

  if(subPopulations_) {
    Population temp(pop.getNumSubPops());
    std::vector<RampTrajectory> trajecs = pop.getBestFromSubPops();
    for(uint8_t i=0;i<trajecs.size();i++) {
      temp.add(trajecs.at(i));
    }

    temp.calcBestIndex();
    msg = temp.populationMsg();
  }
  else {
    msg = pop.populationMsg();
  }

  msg.robot_id = id_;

  msg.population.push_back(movingOn_.msg_);
  h_control_->sendPopulation(msg);
}

void Planner::displayTrajectory(const ramp_msgs::RampTrajectory traj) const 
{
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  h_control_->sendPopulation(pop);
}





/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
const RampTrajectory Planner::evaluateTrajectory(const RampTrajectory trajec) 
{
  //ROS_INFO("In Planner::evaluateTrajectory");
  ros::Time start = ros::Time::now();

  RampTrajectory result = requestEvaluation(trajec);
  ////ROS_INFO("result: %s", result.toString().c_str());
  eval_durs_.push_back( ros::Time::now() - start );

  //ROS_INFO("Leaving Planner::evaluateTrajectory");
  return result;
} // End evaluateTrajectory



/** 
 * This method evaluates each trajectory in the population
 * It also sets i_best_prev_
 **/
const Population Planner::evaluatePopulation(const Population pop) 
{
  //ROS_INFO("In Planner::evaluatePopulation");
  Population result = pop;
  //ROS_INFO("Before evaluating, pop: %s", pop.fitnessFeasibleToString().c_str());
  
  // Go through each trajectory in the population and evaluate it
  for(uint16_t i=0;i<result.size();i++) 
  {
    ////ROS_INFO("i: %i", (int)i);
    result.replace(i, evaluateTrajectory(result.get(i)));
  } // end for
  //ROS_INFO("After evaluating, pop: %s", result.fitnessFeasibleToString().c_str());
 

  i_best_prev_ = result.calcBestIndex();
  
  //ROS_INFO("Exiting Planner::evaluatePopulation");
  return result;
} // End evaluatePopulation





const std::string Planner::pathsToString() const {
  std::ostringstream result;

  result<<"\nPaths:";
  for(unsigned int i=0;i<population_.paths_.size();i++) {
    result<<"\n  "<<population_.paths_.at(i).toString();
  }
  result<<"\n";
  return result.str();
}




const MotionState Planner::findAverageDiff() {
  MotionState result(SP_LU_diffs_.at(0));

  for(uint16_t i=1;i<SP_LU_diffs_.size();i++) {
    result = result.add(SP_LU_diffs_.at(i).abs());
  }

  result = result.divide(SP_LU_diffs_.size());

  return result;
}


void Planner::reportData() 
{
  double sum = 0.;
  for(uint16_t i=0;i<adapt_durs_.size();i++)
  {
    ROS_INFO("Adaptation duration[i]: %f", adapt_durs_.at(i).toSec());
    sum += adapt_durs_.at(i).toSec();
  }
  avg_adapt_dur_ = sum / adapt_durs_.size();
  ROS_INFO("Average adaptation duration: %f", avg_adapt_dur_);


  sum = 0.;
  for(uint16_t i=0;i<trans_durs_.size();i++)
  {
    ROS_INFO("transition duration[i]: %f", trans_durs_.at(i).toSec());
    sum += trans_durs_.at(i).toSec();
  }
  avg_trans_dur_ = sum / trans_durs_.size();
  ROS_INFO("Average transition duration: %f", avg_trans_dur_);

  sum = 0.;
  for(uint16_t i=0;i<cc_durs_.size();i++)
  {
    ROS_INFO("cc duration[i]: %f", cc_durs_.at(i).toSec());
    sum += cc_durs_.at(i).toSec();
  }
  avg_cc_dur_ = sum / cc_durs_.size();
  ROS_INFO("Average cc duration: %f", avg_cc_dur_);

  sum = 0.;
  for(uint16_t i=0;i<mutate_durs_.size();i++)
  {
    ROS_INFO("mutate duration[i]: %f", mutate_durs_.at(i).toSec());
    sum += mutate_durs_.at(i).toSec();
  }
  avg_mutate_dur_ = sum / mutate_durs_.size();
  ROS_INFO("Average mutate duration: %f", avg_mutate_dur_);

  sum = 0.;
  for(uint16_t i=0;i<error_correct_durs_.size();i++)
  {
    ROS_INFO("error correct duration[i]: %f", error_correct_durs_.at(i).toSec());
    sum += error_correct_durs_.at(i).toSec();
  }
  avg_error_correct_dur_ = sum / error_correct_durs_.size();
  ROS_INFO("Average error_correct duration: %f", avg_error_correct_dur_);

  sum = 0.;
  for(uint16_t i=0;i<pc_durs_.size();i++)
  {
    ROS_INFO("pc duration[i]: %f", pc_durs_.at(i).toSec());
    sum += pc_durs_.at(i).toSec();
  }
  avg_pc_dur_ = sum / pc_durs_.size();
  ROS_INFO("Average pc duration: %f", avg_pc_dur_);


  sum = 0.;
  for(uint16_t i=0;i<sc_durs_.size();i++)
  {
    ROS_INFO("sc duration[i]: %f", sc_durs_.at(i).toSec());
    sum += sc_durs_.at(i).toSec();
  }
  avg_sc_dur_ = sum / sc_durs_.size();
  ROS_INFO("Average sc duration: %f", avg_sc_dur_);


  sum = 0.;
  for(uint16_t i=0;i<trajec_durs_.size();i++)
  {
    if(i % 5 == 0)
    {
      ROS_INFO("trajec duration[%i]: %f", i, trajec_durs_.at(i).toSec());
    }
    sum += trajec_durs_.at(i).toSec();
  }
  avg_trajec_dur_ = sum / trajec_durs_.size();
  ROS_INFO("Average trajec duration: %f", avg_trajec_dur_);


  sum = 0.;
  for(uint16_t i=0;i<eval_durs_.size();i++)
  {
    if(i % 20 == 0)
    {
      ROS_INFO("eval duration[i]: %f", eval_durs_.at(i).toSec());
    }
    sum += eval_durs_.at(i).toSec();
  }
  avg_eval_dur_ = sum / eval_durs_.size();
  ROS_INFO("Average eval duration: %f", avg_eval_dur_);


  sum = 0.;
  for(uint16_t i=0;i<error_correct_val_pos_.size();i++)
  {
    ROS_INFO("error_correct_pos[i]: %f", error_correct_val_pos_.at(i));
    sum += error_correct_val_pos_.at(i);
  }
  avg_error_correct_val_pos_ = sum / error_correct_val_pos_.size();
  ROS_INFO("Average position error: %f", avg_error_correct_val_pos_);


  sum = 0.;
  for(uint16_t i=0;i<error_correct_val_or_.size();i++)
  {
    ROS_INFO("error_correct_or[i]: %f", error_correct_val_or_.at(i));
    sum += error_correct_val_or_.at(i);
  }
  avg_error_correct_val_or_ = sum / error_correct_val_or_.size();
  ROS_INFO("Average orientation error: %f", avg_error_correct_val_or_);
}


/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


void Planner::go() 
{

  // t=0
  generation_ = 0;
  
  // initialize population
  initPopulation();
  sendPopulation(population_);
  std::cout<<"\nPopulation initialized! Press enter to continue\n";
  //std::cin.get();
 


  if(seedPopulation_) 
  {
    std::cout<<"\nSeeding population\n";
    seedPopulation();
    i_best_prev_ = population_.calcBestIndex();
    std::cout<<"\nPopulation seeded!\n";
    std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";
    std::cout<<"\n** Pop **:"<<population_.toString();

    // Evaluate after seeding
    population_ = evaluatePopulation(population_);

    // Set movingOn
    movingOn_ = population_.get(population_.calcBestIndex()).getSubTrajectory(controlCycle_.toSec());
    //ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
    

    sendPopulation(population_);
    std::cout<<"\nPopulation seeded! Press enter to continue\n";
    std::cin.get();
  }


  // Create sub-pops if enabled
  if(subPopulations_) 
  {
    population_.createSubPopulations();
    std::cout<<"\nSub-populations created\n";
  }

  // Initialize transPopulation
  transPopulation_ = population_;

  population_at_cc_       = population_;
  transPopulation_at_cc_  = transPopulation_;

  t_start_ = ros::Time::now();

  ROS_INFO("Planning Cycles started!");

  // Start the planning cycles
  //planningCycleTimer_.start();
    
  
  h_parameters_.setCCStarted(false); 


  int num_pc = generationsBeforeCC_ - generationsPerCC_;
  if(num_pc < 0)
  {
    //ROS_WARN("num_pc is less than zero: %i - Setting num_pc = 0", num_pc);
    num_pc = 0;
  }
  ROS_INFO("generationsBeforeCC_: %i generationsPerCC_: %i num_pc: %i", generationsBeforeCC_, generationsPerCC_, num_pc);

  // Wait for the specified number of generations before starting CC's
  //while(generation_ < num_pc) {ros::spinOnce();}
 
  ROS_INFO("Starting CCs at t: %f", ros::Time::now().toSec());

  // Right before starting CC, make sure transPopulation is updated
  transPopulation_ = population_;
  
  // Start the control cycles
  //controlCycleTimer_.start();
  //imminentCollisionTimer_.start();

  //ROS_INFO("CCs started");
 

  /*
   ************************************
   *  Begin planner
   ************************************
   */
  goalThreshold_ = 0.5;
  ros::Rate r(20);
  while( (latestUpdate_.comparePosition(goal_, false) > goalThreshold_) && ros::ok()) 
  {
    planningCycleCallback();
    //r.sleep();
    ros::spinOnce(); 
  } // end while
  /*
   ************************************
   *  Planning done
   ************************************
   */

  ROS_INFO("latestUpdate_: %s\ngoal: %s", latestUpdate_.toString().c_str(), goal_.toString().c_str());


  // Report time stats
  reportData();

  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::RampTrajectory empty;
  h_control_->send(empty);
  h_control_->send(empty);
  h_control_->send(empty);
 
 
  ROS_INFO("Total number of planning cycles: %i", generation_-1);
  ROS_INFO("Total number of control cycles:  %i", num_cc_);
  ROS_INFO("Exiting Planner::go");
} // End go
