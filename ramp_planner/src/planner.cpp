#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(5), populationSize_(5), generation_(0), mutex_start_(true), mutex_pop_(true), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(2.f), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), move_(false)
{
  controlCycle_ = ros::Duration(1.f / 30.f);
  planningCycle_ = ros::Duration(1.f / 50.f);
}

Planner::Planner(const unsigned int r, const int p) : resolutionRate_(r), populationSize_(p), mutex_start_(true), mutex_pop_(true), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(2.f), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), move_(false)
{
  controlCycle_ = ros::Duration(1.f / 30.f);
  planningCycle_ = ros::Duration(1.f / 50.f);
}

Planner::Planner(const ros::NodeHandle& h) : resolutionRate_(5), populationSize_(5), generation_(0), mutex_start_(true), mutex_pop_(true), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(2.f), move_(false)
{
  controlCycle_ = ros::Duration(1.f / 30.f);
  planningCycle_ = ros::Duration(1.f / 50.f);
  init(h); 
}

Planner::~Planner() {
  if(h_traj_req_!= 0) {
    delete h_traj_req_;  
    h_traj_req_= 0;
  }

  if(h_control_ != 0) {
    delete h_control_;
    h_control_ = 0;
  }

  if(h_eval_req_ != 0) {
    delete h_eval_req_;
    h_eval_req_ = 0;
  }
  
  if(modifier_!= 0) {
    delete modifier_;  
    modifier_= 0;
  }
}



/*****************************************************
 ********************** Methods **********************
 *****************************************************/

/** This method initializes the T_od_w_ transform object */
void Planner::setT_od_w(std::vector<float> od_info) {
 
  T_od_w_.setRotation(tf::createQuaternionFromYaw(od_info.at(2)));
  T_od_w_.setOrigin(tf::Vector3(od_info.at(0), od_info.at(1), 0));
} // End setT_od_w


/** Returns an id for RampTrajectory objects */
unsigned int Planner::getIRT() { return i_rt++; }


/** Getter method for start_. It waits for mutex_start_ to be true before returning. */
Configuration Planner::getStartConfiguration() {
  while(!mutex_start_) {}
  return start_;
} // End getStartConfiguration


/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) {
  if(!bestTrajec_.feasible_ && (bestTrajec_.time_until_collision_ < D_)) {
  } 
}




/** 
 * Sets start_ member by taking in the latest update 
 * and transforming it by T_od_w because 
 * updates are relative to odometry frame
 * */
void Planner::updateCallback(const ramp_msgs::Update::ConstPtr& msg) {
  //std::cout<<"\nReceived update!\n";
  
  // Wait for mutex to be true
  while(!mutex_start_) {}

  mutex_start_ = false;

  // Create a Configuration from the update msg 
  Configuration c_od(msg->configuration);

  // Transform configuration from odometry to world coordinates
  c_od.transformBase(T_od_w_);

  // if statement Necessary?
  if(c_od.K_.size() > 0) {
    start_ = c_od;
  }

  //std::cout<<"\nNew starting configuration: "<<start_.toString();

  mutex_start_ = true;
} // End updateCallback


/****************************************************
 ************** Initialization Methods **************
 ****************************************************/


/** Initialize the handlers and allocate them on the heap */
void Planner::init(const ros::NodeHandle& h) {

  // Initialize the handlers
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
  modifier_   = new Modifier(h, paths_, velocities_, num_ops_);

  // Initialize the timers, but don't start them yet
  controlCycleTimer_ = h.createTimer(ros::Duration(controlCycle_), &Planner::controlCycleCallback, this);
  controlCycleTimer_.stop();

  planningCycleTimer_ = h.createTimer(ros::Duration(planningCycle_), &Planner::planningCycleCallback, this);
  planningCycleTimer_.stop();
} // End init

void Planner::planningCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nPlanning cycle occurring, generation = "<<generation_<<"\n";
  
  // Wait until mutex can be obtained
  while(!mutex_pop_) {}
    
  mutex_pop_ = false;
  
  // Call modification
  modification();
  
  mutex_pop_ = true;
  
  // t=t+1
  generation_++;
} // End planningCycleCallback


/* This method returns a trajectory to gradually move into a new trajectory */
void Planner::gradualTrajectory(RampTrajectory& t) {

  // Find where linear velocity starts
  int i=0;
  while(t.msg_trajec_.trajectory.points.at(i).velocities.at(0) == 0 &&
        t.msg_trajec_.trajectory.points.at(i).velocities.at(1) == 0 &&
        i < t.msg_trajec_.trajectory.points.size()) {i++;}

  // If i is in the trajectory (it won't be if rotation-only) 
  if( i < t.msg_trajec_.trajectory.points.size()) {

    // Get the point
    trajectory_msgs::JointTrajectoryPoint p = t.msg_trajec_.trajectory.points.at(i);

    // Set the linear velocities to be the same as p
    for(unsigned int j=0;j<i;j++) {
      t.msg_trajec_.trajectory.points.at(j).velocities.at(0) = p.velocities.at(0);
      t.msg_trajec_.trajectory.points.at(j).velocities.at(1) = p.velocities.at(1);
    } //end for
  } //end if i in range
} // End gradualTrajectory


/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {

  // std::cout<<"\nSpinning once\n";
  ros::spinOnce(); 
  
  // Obtain mutex on population and update it
  while(!mutex_pop_) {}
  mutex_pop_ = false;
  updatePopulation(controlCycle_);
  mutex_pop_ = true;

  /** TODO **/
  // Create subpopulations in P(t)


  // Evaluate P(t) and obtain best T, T_move=T_best
  bestTrajec_ = evaluateAndObtainBest();


  // Find orientation difference between the old and new trajectory 
  // old configuration is the new start_ configuration (last point on old trajectory)
  // new orientation is the amount to rotate towards first knot point
  float b = u.findAngleFromAToB(start_.K_, bestTrajec_.path_.all_.at(1).configuration_.K_);
  float diff = u.findDistanceBetweenAngles(start_.K_.at(2), b);
  if(fabs(diff) <= 0.52356f) {
    gradualTrajectory(bestTrajec_);
  }

  
  // Send the best trajectory 
  sendBest(); 
} // End controlCycleCallback




/** This function generates the initial population of trajectories */
void Planner::init_population() { 
  
  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path(start_, goal_);

    // Each trajectory will have a random number of knot points
    // Put a max of 5 knot points for practicality...
    unsigned int num = rand() % 5;

    // For each knot point to be created...
    for(unsigned int j=0;j<num;j++) {

      // Create a random configuration
      Configuration temp_config;
      temp_config.ranges_ = ranges_;
      temp_config.random();
      
      // Add the random configuration to the path
      temp_path.Add(temp_config); 
    }

    // Add the path to the list of paths
    paths_.push_back(temp_path);
  }
  
  std::vector<float> v;
  // For each path
  for(unsigned int i=0;i<paths_.size();i++) {

    // Hardcode some velocities, 0.25m/s per segment
    for(unsigned int j=1;j<paths_.at(i).all_.size();j++) {
      v.push_back(0.35f);
    }
    
    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i,v,v);
    
    // Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(getIRT());
      
      // Set the Trajectory msg
      temp.msg_trajec_ = msg_request.response.trajectory;

      // Set trajectory path
      temp.path_ = paths_.at(i);
      
      // Add the trajectory to the population
      population_.add(temp);
    }
    else {
      // some error handling
    }

    // Push the times vector onto velocities_
    velocities_.push_back(v);

    // Clear times vector
    v.clear();
  } // end for


  // Update the modifier
  modifier_->updateAll(paths_, velocities_);
} // End init_population





/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
bool Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr) {
  return h_traj_req_->request(tr); 
}

/** Request an evaluation */
bool Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) {
  return h_eval_req_->request(er);
}



/******************************************************
 ****************** Modifying Methods *****************
 ******************************************************/


/** Modify a Path */
const std::vector<Path> Planner::modifyPath() { 
  return modifier_->perform();
}

/** Modify a trajectory */ 
const std::vector<ModifiedTrajectory> Planner::modifyTrajec() {
  //std::cout<<"\nIn modifyTrajec\n";
  std::vector<ModifiedTrajectory> result;
  

  // The modification operators deal with paths
  // So send the path to be modified
  std::vector<Path> modded_paths = modifyPath();

  // Hold the ids of the path(s) modified
  std::vector<int> olds;
  olds.push_back(modifier_->i_changed1);

  // If a crossover was performed, push on the 2nd path 
  if(modded_paths.size()>1) {
    olds.push_back(modifier_->i_changed2);
  }

  // Hold the new velocity vector(s)
  std::vector< std::vector<float> > vs = getNewVelocities(modded_paths, olds); 

  
  // For each targeted path,
  for(unsigned int i=0;i<modded_paths.size();i++) {
    
    // Build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(modded_paths.at(i), vs.at(i), vs.at(i));
    
    // Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
      // Build RampTrajectory
      RampTrajectory temp(getIRT());
      temp.msg_trajec_ = tr.response.trajectory;
      temp.path_ = modded_paths.at(i);

      // Build ModifiedTrajectory
      ModifiedTrajectory mt;
      mt.trajec_ = temp;
      mt.velocities_ = vs.at(i); 

      result.push_back(mt);
    } // end if
  } // end for
  
  return result;
} // End modifyTrajectory


/******************************************************
 **************** Srv Building Methods ****************
 ******************************************************/



/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e ) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.v_start        = v_s;
  result.request.v_end          = v_e;
  result.request.resolutionRate = resolutionRate_;

  return result;
} // End buildTrajectoryRequest


/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = paths_.at(i_path).buildPathMsg();
  result.request.v_start        = v_s;
  result.request.v_end          = v_e;
  result.request.resolutionRate = resolutionRate_;
  
  return result; 
} // End buildTrajectoryRequest


/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const unsigned int i_trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = population_.population_.at(i_trajec).msg_trajec_;

  return result;
} // End buildEvaluationRequest

/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = trajec.msg_trajec_;

  return result;
} // End buildEvaluationRequest



/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/

/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {

  // If infeasible and too close to obstacle, 
  // Stop the robot by sending a blank trajectory
  if(!bestTrajec_.feasible_ && (bestTrajec_.time_until_collision_ < 2.f)) {
    std::cout<<"\nCollision within 2 seconds! Stopping robot!\n";
    ramp_msgs::Trajectory blank;
    h_control_->send(blank); 
  }
  else if(!bestTrajec_.feasible_) {
    std::cout<<"\nBest trajectory is not feasible! Time until collision: "<<bestTrajec_.time_until_collision_;
    h_control_->send(bestTrajec_.msg_trajec_);
  }
  else {
    h_control_->send(bestTrajec_.msg_trajec_);
  }
} // End sendBest




/** Send the whole population of trajectories to the trajectory viewer */
void Planner::sendPopulation() {

  // Need to set robot id
  ramp_msgs::Population msg = population_.populationMsg();
  msg.robot_id = id_;

  h_control_->sendPopulation(msg);
}



/** 
 * This method returns the new velocity vector(s) for the modified path(s) 
 * This is called at each control cycle to update the trajectories
 * */
const std::vector< std::vector<float> > Planner::getNewVelocities(std::vector<Path> new_paths, std::vector<int> i_old) {
  //std::cout<<"\nIn getNewVelocities\n";

  std::vector< std::vector<float> > result; 

  // If the modification only changed one path, i.e., not a crossover 
  if(new_paths.size() == 1) {
    // std::cout<<"\nOnly changed one path!\n";
    // std::cout<<"\ni_old.at(0):"<<i_old.at(0);

    // Get the original velocity values
    std::vector<float> v = velocities_.at(i_old.at(0));

    // Get the old and new paths
    Path old = paths_.at(i_old.at(0));
    Path new_path = new_paths.at(0);
    // std::cout<<"\nold:"<<old.toString();
    // std::cout<<"\nnew_path:"<<new_path.toString();

    // Check the difference in path sizes
    int diff = new_path.all_.size() - old.all_.size();
    // std::cout<<"\ndiff:"<<diff<<"\n";

    // Insertion
    if(diff == 1) {
      v.push_back(v.at(0));
    } // end if insertion 

    // Deletion
    else if(diff == -1) {
      v.erase(v.begin());
    } // end if deletion

    // Push onto the result
    result.push_back(v);
  } // end if not crossover


  // Crossover
  else {
    
    // For each path,
    for(unsigned int i=0;i<new_paths.size();i++) {
    
      // Get the original velocity values
      std::vector<float> v = velocities_.at(i_old.at(i));
      
      // Get the old and new paths
      Path old = paths_.at(i_old.at(i)); 
      Path new_path = new_paths.at(i);
      // std::cout<<"\nold path "<<i<<":"<<old.toString();
      // std::cout<<"\nnew path "<<i<<":"<<new_path.toString();

      int diff = new_path.all_.size() - old.all_.size();
      
      if(diff > 0) {
        for(unsigned int d=0;d<diff;d++) {
          v.push_back(v.at(0)); 
        }
      }
      else {
        for(int d=diff-1;d>0;d--) {
          v.erase(v.begin());
        }
      } // end else

      result.push_back(v);
    } // end for
  } // end else
  
  // Swap and Change do not change the size

  return result;
} // End getNewVelocities



/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  and set tau to the new best */
void Planner::modification() {

  // Modify 1 or more trajectories
  std::vector<ModifiedTrajectory> mod_trajec = modifyTrajec();
  
  // Add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {

    // Evaluate the new trajectory
    evaluateTrajectory(mod_trajec.at(i).trajec_);
    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i).trajec_);

    // Update the path and velocities in the planner 
    paths_.at(index)      = mod_trajec.at(i).trajec_.path_;
    velocities_.at(index) = mod_trajec.at(i).velocities_;
    
    // Update the path and velocities in the modifier 
    modifier_->update(paths_.at(index), index);
  } // end for
  
  // Obtain and set best trajectory
  bestTrajec_ = population_.findBest();
  
  // Send the whole population to the trajectory viewer
  sendPopulation();
} // End modification


/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
void Planner::evaluateTrajectory(RampTrajectory& trajec) {

  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(trajec);
  
  // Do the evaluation and set the fitness and feasibility members
  if(requestEvaluation(er)) {
    trajec.fitness_   = er.response.fitness;
    trajec.feasible_  = er.response.feasible;
    trajec.msg_trajec_.fitness    = trajec.fitness_;
    trajec.msg_trajec_.feasible   = trajec.feasible_;
    trajec.time_until_collision_  = er.response.time_until_collision;
  }
  else {
    // TODO: some error handling
  }
} // End evaluateTrajectory


void Planner::evaluatePopulation() {
  
  // Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<population_.population_.size();i++) {
    evaluateTrajectory(population_.population_.at(i));
  } // end for   
} // End evaluatePopulation



/** This method updates all the paths with the current configuration */
void Planner::updatePaths(Configuration start, ros::Duration dur) {
  // std::cout<<"\nUpdating start to: "<<start.toString();
  // std::cout<<"\ndur: "<<dur<<"\n";


  // For each trajectory
  for(unsigned int i=0;i<population_.population_.size();i++) {

    // Track how many knot points we get rid of
    unsigned int throwaway=0;

    // For each knot point,
    for(unsigned int i_kp=0;i_kp<population_.population_.at(i).msg_trajec_.index_knot_points.size();i_kp++) {
     
      // Get the point 
      trajectory_msgs::JointTrajectoryPoint point = population_.population_.at(i).msg_trajec_.trajectory.points.at( population_.population_.at(i).msg_trajec_.index_knot_points.at(i_kp));
      // std::cout<<"\npoint["<<i<<"].time_from_start:"<<point.time_from_start;

      // Compare the durations
      if( dur > point.time_from_start) {
        throwaway++;
      }
    } //end for

    // If the whole path has been passed, adjust throwaway so that 
    //  we are left with a path that is: {new_start_, goal_}
    if( throwaway >= paths_.at(i).size() ) { 
      throwaway = paths_.at(i).size()-1;
    } 
    
    // Erase the amount of throwaway points (points we have already passed)
    paths_.at(i).all_.erase( paths_.at(i).all_.begin(), paths_.at(i).all_.begin()+throwaway );
    
    // Insert the new starting configuration
    paths_.at(i).all_.insert( paths_.at(i).all_.begin(), start_);

    // Set start_ to be the new starting configuration of the path
    paths_.at(i).start_ = start;

    // Update the modifier
    modifier_->update(paths_.at(i), i);

    // Also erase velocities, throwaway-1
    if(throwaway > 0) {
      velocities_.at(i).erase( velocities_.at(i).begin(), velocities_.at(i).begin()+throwaway-1);
    }
  } // end for
} // End updatePaths


/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::updatePopulation(ros::Duration d) {
  
  // First, get the updated current configuration
  start_ = getStartConfiguration();
  
  // Update the paths with the new starting configuration 
  updatePaths(getStartConfiguration(), d);

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(unsigned int i=0;i<paths_.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i, velocities_.at(i), velocities_.at(i));
    
    // Send the request 
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(population_.population_.at(i).id_);
      
      // Set the Trajectory msg and the path
      temp.msg_trajec_  = msg_request.response.trajectory;
      temp.path_        = paths_.at(i);
      
      // Push onto updatedTrajecs
      updatedTrajecs.push_back(temp);
    } // end if
  } // end for

  // Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);
} // End updatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest() {
  evaluatePopulation();
  return population_.findBest();
}



const std::string Planner::pathsToString() const {
  std::ostringstream result;

  result<<"\nPaths:";
  for(unsigned int i=0;i<paths_.size();i++) {
    result<<"\n  "<<paths_.at(i).toString();
  }
  result<<"\n";
  return result.str();
}





/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


 void Planner::go() {

  // t=0
  generation_ = 0;
  
  // initialize population
  init_population();
  
  /*std::cout<<"\nPopulation initialized!\n";
  for(unsigned int i=0;i<paths_.size();i++) {
    std::cout<<"\nPath "<<i<<": "<<paths_.at(i).toString();
  }*/
  // std::cout<<"\nPress enter to continue\n";
  // std::cin.get();


  // Initialize the modifier
  modifier_->paths_ = paths_;


  // Evaluate the population and get the initial trajectory to move on
  RampTrajectory T_move = evaluateAndObtainBest();
  std::cout<<"\nPopulation evaluated!\n"<<population_.fitnessFeasibleToString()<<"\n\n"; 
  sendPopulation();
  std::cout<<"\nAfter sendPopulation\n";
  // std::cout<<"\nPress enter to start the loop!\n";
  // std::cin.get();
  

  /** TODO */
  // createSubpopulations();
  
  // Start the planning cycle timer
  planningCycleTimer_.start();

  // Wait for 75 generations before starting control cycle
  while(generation_ < 100) {ros::spinOnce();}

  std::cout<<"\n***************Starting Control Cycle*****************";
  // Start the control cycle timer
  controlCycleTimer_.start();
  
  // Do planning until robot has reached goal
  // D = 0.4 if considering mobile base, 0.2 otherwise
  goalThreshold_ = 0.1;
  while( (start_.compare(goal_, false) > goalThreshold_) && ros::ok()) {
    ros::spinOnce(); 
  } // end while
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();

  // Send an empty trajectory
  ramp_msgs::Trajectory empty;
  h_control_->send(empty);
  
  std::cout<<"\nPlanning complete!\n";
} // End go
