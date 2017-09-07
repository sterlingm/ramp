#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "evaluation_request_handler.h"
#include "trajectory_request_handler.h"
#include "nav_msgs/Odometry.h"
#include "modifier.h"
#include "population.h"
#include "control_handler.h"
#include "parameter_handler.h"
#include "bezier_curve.h"
#include <type_traits>

struct ModificationResult 
{
  Population popNew_;
  Population transNew_;
  std::vector<uint16_t> i_modified_;
};


enum MotionType 
{
  MT_NONE                         = 0,
  MT_TRANSLATION                  = 1,
  MT_ROTATION                     = 2,
  MT_TRANSLATON_AND_ROTATION      = 3,
};


class Planner {
  public:
    Planner();
    ~Planner();


    /*******************************************
     ************** Data Members ***************
     *******************************************/
    

    // Hold the population of trajectories, 
    // the velocities of each trajectory's segments,
    // the trajectory's path,
    // and the resolution rate for the trajectories
    Population                          population_;
    Population                          population_at_cc_;
    const float                         resolutionRate_;
    
    // Hold the start and goal configurations
    // and the ranges for each DOF
    MotionState start_;
    MotionState goal_;
    std::vector<Range> ranges_;

    // Starting motion state for planning cycle
    MotionState startPlanning_;

    MotionState latestUpdate_;
    
    
    // The most fit trajectory in the population


    // Timer for sending the best trajec
    // Control cycle - used for determining when to update P(t)
    ros::Timer    controlCycleTimer_;
    ros::Duration controlCycle_;
    
    // Timer for doing a modification
    // Planning cycle - used for determining when to update P(t)
    ros::Timer    planningCycleTimer_;
    ros::Duration planningCycle_;
    
    // Sensing cycle
    ros::Duration sensingCycle_;

    // Cycle to check imminent collision
    ros::Timer imminentCollisionTimer_;
    ros::Duration imminentCollisionCycle_;
   
    
    // Robot ID
    int id_;

    std::vector<double> COLL_DISTS;
    int i_COLL_DISTS_;

    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/


    
    // Start planning
    void go();
    
    // Initialization 
    void initPopulation();
    void init(const uint8_t             i,                
              const ros::NodeHandle&    h, 
              const MotionState         s,                
              const MotionState         g, 
              const std::vector<Range>  r,                
              const int                 population_size, 
              const bool                sub_populations,  
              const std::vector<tf::Transform> ob_T_odoms,
              const TrajectoryType      pop_type=HYBRID,
              const int                 gens_before_cc=0,
              const double              t_pc_rate=2.,
              const double              t_fixed_cc=2.,
              const bool                errorReduction=0);
    
    // Send the best trajectory to the control package
    void sendBest();
    
    // Send the whole population to the trajectory viewer
    void sendPopulation(const Population pop) const;
    void displayTrajectory(const ramp_msgs::RampTrajectory traj) const;

    // Evaluate the population 
    const RampTrajectory  evaluateTrajectory(const RampTrajectory trajec);
    const Population      evaluatePopulation(const Population pop);
    void evaluateTrajectoryOOP(RampTrajectory& t) const;
    void evaluatePopulationOOP();
    
    // Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<RampTrajectory> modifyTrajec();



    // Msg building methods 
    const ramp_msgs::TrajectorySrv buildTrajectorySrv(
      const Path path, const std::vector<ramp_msgs::BezierCurve> curves,
                    const int id=-1) const;
    const ramp_msgs::TrajectorySrv buildTrajectorySrv(
        const Path path, const int id=0) const;

    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const Path path, const std::vector<ramp_msgs::BezierCurve> curves,
              const int id=-1) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const Path path, const int id=0) const;



    const ramp_msgs::EvaluationSrv buildEvaluationSrv(
              const RampTrajectory trajec)      ;
    const ramp_msgs::EvaluationSrv buildEvaluationSrv(
              const std::vector<RampTrajectory> trajecs)      ;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(
              const RampTrajectory trajec)      const;


    void buildEvaluationSrvOOP(std::vector<RampTrajectory>& trajecs, ramp_msgs::EvaluationSrv& result) const;
    void buildEvaluationSrvOOP(const RampTrajectory& trajec, ramp_msgs::EvaluationSrv& result) const;
    void buildEvaluationRequestOOP(const RampTrajectory& trajec, ramp_msgs::EvaluationRequest& result) const;

    void requestEvaluationOOP(std::vector<RampTrajectory>& trajecs) const;
    void requestEvaluationOOP(ramp_msgs::EvaluationRequest& request) const;
    void requestEvaluationOOP(RampTrajectory& t) const;


    // Request information from other packages
    // Cannot make the request srvs const because they have no serialize/deserialize
    const std::vector<RampTrajectory> requestTrajectory(ramp_msgs::TrajectorySrv& tr, const int id=-1);
    const std::vector<RampTrajectory> requestTrajectory(std::vector<ramp_msgs::TrajectoryRequest> trs); 
    const RampTrajectory requestTrajectory(const Path p, const int id=-1);
    const RampTrajectory requestTrajectory(ramp_msgs::TrajectoryRequest tr);

    const std::vector<RampTrajectory> requestEvaluation(std::vector<RampTrajectory> trajecs);
    const RampTrajectory requestEvaluation(ramp_msgs::EvaluationRequest& er);
    const RampTrajectory requestEvaluation(const RampTrajectory traj);


    // Update the population 
    const Population adaptPopulation( const Population pop, 
                                      const MotionState ms, 
                                      const ros::Duration d );

    // Display all of the paths
    const std::string pathsToString() const;

    // Set the transformation from odometry to world CS
    void setT_base_w(std::vector<double> base_pos);

    // Sets the m_i vector
    const std::vector<MotionState> setMi(const RampTrajectory trj_current) const;

    void setOb_T_w_odom();
    std::vector<tf::Transform> ob_T_w_odom_;
    std::vector<RampTrajectory> ob_trajectory_;


    const MotionType findMotionType(const ramp_msgs::Obstacle ob) const;
    const ramp_msgs::RampTrajectory getPredictedTrajectory(const ramp_msgs::Obstacle ob, const tf::Transform tf) const;
    const ramp_msgs::Path getObstaclePath(const ramp_msgs::Obstacle ob, const tf::Transform T_w_odom, const MotionType mt) const;
    
    void sensingCycleCallback     (const ramp_msgs::ObstacleList& msg);
    void updateCallback(const ramp_msgs::MotionState& msg);

    /** Data */

    // Motion state that should be reached by next control cycle
    MotionState m_cc_;
    



    // Hold the difference between previous startPlanning 
    // and latestUpdate for each control cycle
    std::vector<MotionState> SP_LU_diffs_;
    const MotionState findAverageDiff();
  

    // Debugging variables
    bool modifications_;
    bool evaluations_;
    bool seedPopulation_;
    
    const double updateCurvePos(const RampTrajectory traj, const ros::Duration d) const;
  //private:
    /** These are (mostly) utility members that are only used by Planner and should not be used by other classes */


    /***** Methods *****/

    const Path getRandomPath(const MotionState s, const MotionState g) const;
    const Path getAdjustedPath(const MotionState s, const MotionState g) const;
    
    // Initialize start and goal
    void initStartGoal(const MotionState s, const MotionState g);
    
    // Updates the paths in P(t) so we can get new trajectories
    const uint8_t getNumThrowawayPoints(const RampTrajectory traj, const ros::Duration dur) const;
    const std::vector<Path>                   adaptPaths( const Population pop,
                                                          const MotionState start, 
                                                          const ros::Duration dur) const;

    const std::vector<ramp_msgs::BezierCurve> adaptCurves( const Population pop,
                                                                          const MotionState ms,
                                                                          const ros::Duration d) const;

    const ramp_msgs::BezierCurve               handleCurveEnd(const RampTrajectory traj) const;

    // Returns a unique id for a RampTrajectory 
    const unsigned int getIRT();

    // Modification procedure
    const ModificationResult modification();

    // Callback methods for ros::Timers
    void controlCycleCallback     (const ros::TimerEvent& t);
    //void planningCycleCallback    (const ros::TimerEvent& t);
    void planningCycleCallback    ();
    void imminentCollisionCallback(const ros::TimerEvent& t);
    
    
    
   
    // Misc
    const MotionState randomizeMSPositions(const MotionState ms)        const ;
          void checkTrajChange()                                        ;
          void seedPopulation()                                         ;
          void seedPopulationTwo()                                      ;

    const RampTrajectory  getTransitionTrajectory(const RampTrajectory movingOn, 
                                                  const RampTrajectory trgt_traj,
                                                  const double t);
    const RampTrajectory getBestTransTrajectory(const RampTrajectory moving,
                                                const RampTrajectory target);



    const MotionState     errorCorrection();



    const std::vector<RampTrajectory> switchTrajectory( const RampTrajectory from, 
                                                        const RampTrajectory to );
    const RampTrajectory computeFullSwitch(const RampTrajectory from, const RampTrajectory to);
    const bool checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to)
      const;
    
   





    const ramp_msgs::BezierCurve replanCurve(const RampTrajectory trajec, const MotionState ms_start) const;
    const RampTrajectory replanTrajec(const RampTrajectory trajec, const MotionState ms_start);
    const std::vector<RampTrajectory> replanTrajecs(const std::vector<RampTrajectory> trajecs, const MotionState ms_start);
    const std::vector<RampTrajectory> getTrajectories(const std::vector<Path> p);
    const std::vector<RampTrajectory> getTrajectories(std::vector<ramp_msgs::TrajectoryRequest> tr);
    void updatePathsStart(const MotionState s);




    // 1 if before, 2 if on curve, 3 if past curve 
    const int estimateIfOnCurve(const MotionState ms, 
                                const ramp_msgs::BezierCurve curve) const;

    const std::vector<Path> getRandomPaths        ( const MotionState init, const MotionState goal);
    const std::vector<Path> getAdjustedPaths      ( const MotionState init, const MotionState goal);
    const Population        getPopulation         ( const MotionState init, 
                                                    const MotionState goal, 
                                                    const bool        random = false );


    // Work for CC
    void doControlCycle();
    const uint8_t computeSwitchPC(const Population pop, const RampTrajectory moving);
    const uint8_t computeSwitchPC(const RampTrajectory target, const RampTrajectory moving);

    // Returns the index in the trajectory's path to start checking if the robot has passed it
    const uint8_t getIndexStartPathAdapting(const RampTrajectory t) const;


    // Returns true if motion state satisfies constraints to be a knot point in Path p 
    const bool validKPForPath(const MotionState ms, const Path p) const;

    // Methods for debugging
    void stopForDebugging();
    void restartAfterDebugging();
    void pause();


    const Population getTransPop(const Population pop, const RampTrajectory movingOn);


    const RampTrajectory offsetTrajectory(const RampTrajectory t, const MotionState diff) const;
    const Population offsetPopulation(const Population pop, const MotionState diff) const;

    bool predictTransition(const RampTrajectory from, const RampTrajectory to, const double t);

    void reportData() ;


    ros::Time t_prevIC_;
    ros::Time t_prevObIC_;
    ros::Timer ob_dists_timer_;
    double ob_dists_timer_dur_;
    void obICCallback(const ros::TimerEvent& e);
    std::vector<double> ob_dists_;


    /***** Data members *****/

    // Utility instance
    Utility             utility_; 

    // Size of population
    unsigned int        populationSize_;

    // Generation counter
    unsigned int        generation_;

    // ID counter for trajectories
    unsigned int        i_rt;

    // Last time P(t) was updated
    ros::Time           lastUpdate_;

    // How far we need to get to the goal before stopping
    float               goalThreshold_;

    // Number of modification operators 
    unsigned int        num_ops_;

    // Distance threshold for imminent collision
    float               D_;

    // Index of previous best trajectory
    unsigned int        i_best_prev_;

    // Number of generations to wait before starting control cycles
    unsigned int        generationsBeforeCC_;

    // Maximum number of generations to occur between control cycles
    unsigned int        generationsPerCC_;

    // Flag for if the control cycles have started
    bool                cc_started_;

    // Flag for if sub-populations are being used
    bool                subPopulations_;
    
    // Number of planning cycles since last control cycle
    int                 c_pc_;

    // Threshold for getting transition trajectory
    double              transThreshold_;

    // Number of Control Cycles that have occurred
    int                 num_cc_;

    // Distance between each knot points
    double              L_;

    // Type of trajectories the population will hold
    TrajectoryType      pop_type_;

    // Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    EvaluationRequestHandler*   h_eval_req_;
    ControlHandler*             h_control_;
    Modifier*                   modifier_;

    // Parameter handler
    ParameterHandler            h_parameters_;


    double delta_t_switch_;

    double          t_fixed_cc_;
    RampTrajectory  movingOnCC_;
    RampTrajectory  movingOn_;

    // Error Reduction variables
    bool                      errorReduction_;
    std::vector<MotionState>  m_i_;
    
    // Stop things for debugging
    bool stop_;


    bool imminent_collision_;


    bool moving_on_coll_;


    ros::Time t_prevCC_;
    uint8_t pc_switch_;


    ros::Time t_start_;


    tf::Transform T_w_odom_;

    std::vector<double> error_correct_val_pos_, error_correct_val_or_;
    double avg_error_correct_val_pos_, avg_error_correct_val_or_;
    

    std::vector<ros::Duration> adapt_durs_;
    std::vector<ros::Duration> trans_durs_;
    std::vector<ros::Duration> cc_durs_;
    std::vector<ros::Duration> mutate_durs_;
    std::vector<ros::Duration> pc_durs_;
    std::vector<ros::Duration> sc_durs_;
    std::vector<ros::Duration> trajec_durs_;
    std::vector<ros::Duration> eval_durs_;
    std::vector<ros::Duration> error_correct_durs_;
    double avg_adapt_dur_, avg_trans_dur_, avg_cc_dur_, avg_mutate_dur_, avg_pc_dur_, avg_trajec_dur_, 
           avg_eval_dur_, avg_error_correct_dur_, avg_sc_dur_;


    bool print_enter_exit_;
};

#endif
