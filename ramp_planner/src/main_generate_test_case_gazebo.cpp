#include "planner.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <fstream>

//gaz
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
 

Utility utility;


Planner             my_planner; 
int                 id;
MotionState         start, goal;
std::vector<Range>  ranges;
int                 population_size;
int                 gensBeforeCC;
bool                sub_populations;
bool                modifications;
bool                evaluations;
bool                seedPopulation;
bool                errorReduction;
double              t_cc_rate;
double              t_pc_rate;
int                 num_obs;
int                 pop_type;
TrajectoryType      pt;
std::vector<std::string> ob_topics;
std::vector<tf::Transform> ob_tfs;
ros::Publisher pub_obs, pub_resetOdom;

std::vector<int> ob_delay;

/*
 * Data to collect
 */
ramp_msgs::RampTrajectory bestTrajec;
std::vector<RampTrajectory> bestTrajec_at_end;
int num_IC;
bool IC_occur;
bool IC_current;
bool collAmongObs;
std::vector<bool> colls;
std::vector<bool> icAtColl;
MotionState latestUpdate;

ros::ServiceClient setModelSrv;
ros::ServiceClient getModelSrv;

// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  
  for(unsigned int i=0;i<dof_min.size();i++) {
    Range temp(dof_min.at(i), dof_max.at(i));
    ranges.push_back(temp); 
  }

} // End initDOF



// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) 
{
  for(unsigned int i=0;i<s.size();i++) {
    start.msg_.positions.push_back(s.at(i));
    goal.msg_.positions.push_back(g.at(i));

    start.msg_.velocities.push_back(0);
    goal.msg_.velocities.push_back(0);

    start.msg_.accelerations.push_back(0);
    goal.msg_.accelerations.push_back(0);

    start.msg_.jerks.push_back(0);
    goal.msg_.jerks.push_back(0);
  }
} // End initStartGoal



void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}


/** Loads all of the ros parameters from .yaml 
 *  Calls initDOF, initStartGoal */
void loadParameters(const ros::NodeHandle handle) 
{
  std::cout<<"\nLoading parameters\n";
  std::cout<<"\nHandle NS: "<<handle.getNamespace();

  std::string key;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) 
  {
    handle.getParam("robot_info/id", id);
  }
  else 
  {
    ROS_ERROR("Did not find parameter robot_info/id");
  }


  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    initDOF(dof_min, dof_max);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }


  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> p_start;
    std::vector<float> p_goal;
    handle.getParam("robot_info/start", p_start);
    handle.getParam("robot_info/goal",  p_goal );
    initStartGoal(p_start, p_goal);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
  }



  if(handle.hasParam("ramp/population_size")) 
  {
    handle.getParam("ramp/population_size", population_size);
    std::cout<<"\npopulation_size: "<<population_size;
  }

  
  if(handle.hasParam("ramp/sub_populations")) 
  {
    handle.getParam("ramp/sub_populations", sub_populations);
    std::cout<<"\nsub_populations: "<<sub_populations;
  }
  
  if(handle.hasParam("ramp/modifications")) 
  {
    handle.getParam("ramp/modifications", modifications);
    std::cout<<"\nmodifications: "<<modifications;
  }
  
  if(handle.hasParam("ramp/evaluations")) 
  {
    handle.getParam("ramp/evaluations", evaluations);
    std::cout<<"\nevaluations: "<<evaluations;
  }
  
  if(handle.hasParam("ramp/seed_population")) 
  {
    handle.getParam("ramp/seed_population", seedPopulation);
    std::cout<<"\nseed_population: "<<seedPopulation;
  }
  
  if(handle.hasParam("ramp/gens_before_control_cycle")) 
  {
    handle.getParam("ramp/gens_before_control_cycle", gensBeforeCC);
    std::cout<<"\ngens_before_control_cycle: "<<gensBeforeCC;
  }
  
  if(handle.hasParam("ramp/fixed_control_cycle_rate")) 
  {
    handle.getParam("ramp/fixed_control_cycle_rate", t_cc_rate);
    ROS_INFO("t_cc_rate: %f", t_cc_rate);
  }
  
  if(handle.hasParam("ramp/pop_traj_type")) 
  {
    handle.getParam("ramp/pop_traj_type", pop_type);
    ROS_INFO("pop_type: %s", pop_type ? "Partial Bezier" : "All Straight");
    switch (pop_type) 
    {
      case 0:
        pt = HOLONOMIC;
        break;
      case 1:
        pt = HYBRID;
        break;
    }
  }
  
  if(handle.hasParam("ramp/error_reduction")) 
  {
    handle.getParam("ramp/error_reduction", errorReduction);
    ROS_INFO("errorReduction: %s", errorReduction ? "True" : "False");
  }

  if(handle.hasParam("ramp/num_of_obstacles"))
  {
    handle.getParam("ramp/num_of_obstacles", num_obs);
    ROS_INFO("num_of_obstacles: %i", num_obs);
  }


  if(handle.hasParam("ramp/obstacle_topics"))
  {
    handle.getParam("ramp/obstacle_topics", ob_topics);
    ROS_INFO("ob_topics.size(): %i", (int)ob_topics.size());
    for(int i=0;i<ob_topics.size();i++)
    {
      ROS_INFO("ob_topics[%i]: %s", i, ob_topics.at(i).c_str());
    }
  }



  std::cout<<"\n------- Done loading parameters -------\n";
    std::cout<<"\n  ID: "<<id;
    std::cout<<"\n  Start: "<<start.toString();
    std::cout<<"\n  Goal: "<<goal.toString();
    std::cout<<"\n  Ranges: ";
    for(uint8_t i=0;i<ranges.size();i++) 
    {
      std::cout<<"\n  "<<i<<": "<<ranges.at(i).toString();
    }
  std::cout<<"\n---------------------------------------";
}





enum Group 
{
  EXTERIOR = 0,
  INTERIOR = 1,
  CRITICAL = 2,
  MIX      = 3
};

struct ABTC
{
  bool moving[9];
  double times[9];
};


struct ObInfo
{
  double x;
  double y;
  double v;
  double w;
  double relative_direction;
  double d;
  ramp_msgs::Obstacle msg;
  
  bool faster;
};

struct TestCase {
  ros::Time t_begin;
  std::vector<ObInfo> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<RampTrajectory> ob_trjs;
  Group group;
  int history;

  bool success;
};


struct TestCaseTwo {
  ABTC abtc;
  std::vector<ObInfo> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<ramp_msgs::RampTrajectory> ob_trjs;
  ros::Time t_begin;
};




struct ObInfoExt
{
  // Position details
  double x;
  double y;
  double relative_direction;
  double d;
  
  // Linear speed
  double v_i;
  double v_f;

  // Angular speed
  double w;

  // Durations
  ros::Duration d_s;
  ros::Duration d_vi;
  ros::Duration d_vf;
  
  ramp_msgs::Obstacle msg;

  int last_index=0;
  
  bool faster;
};

struct TestCaseExt
{
  ABTC abtc;
  std::vector<ObInfoExt> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<ramp_msgs::RampTrajectory> ob_trjs;
  ros::Time t_begin;

  ros::Duration d_states;
  // Ext portion
};


TestCaseExt globalTc;


ramp_msgs::Obstacle getStaticOb(ramp_msgs::Obstacle ob)
{
  ramp_msgs::Obstacle result = ob; 

  result.ob_ms.velocities[0] = 0;
  result.ob_ms.velocities[1] = 0;
  result.ob_ms.velocities[2] = 0;

  return result;
}

void setStaticObGazebo(ramp_msgs::Obstacle ob, const std::string name)
{
  // Don't need to build an obstacle msg, need to move in Gazebo
  gazebo_msgs::SetModelState setModelState;
  gazebo_msgs::ModelState modelState; 
  modelState.model_name = name;
  modelState.reference_frame = "map";
  modelState.pose.position.x = ob.ob_ms.positions[0];
  modelState.pose.position.y = ob.ob_ms.positions[1];

  setModelState.request.model_state = modelState;

  if(setModelSrv.call(setModelState))
  {
    ROS_INFO("Set state");
  }
  else
  {
    ROS_INFO("Problem setting state");
  }
}


void resetRobotOdom()
{
  std_msgs::Empty e;
  
  pub_resetOdom.publish(e);
}

void setRobotInitialPos()
{
  // Don't need to build an obstacle msg, need to move in Gazebo
  gazebo_msgs::SetModelState setModelState;
  gazebo_msgs::ModelState modelState; 
  modelState.model_name = "mobile_base";
  modelState.reference_frame = "map";
  modelState.pose.position.x = 0;
  modelState.pose.position.y = 0;

  modelState.twist.linear.x = 0;
  modelState.twist.linear.y = 0;
  modelState.twist.angular.z = 0;

  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.785f), modelState.pose.orientation);

  setModelState.request.model_state = modelState;

  if(setModelSrv.call(setModelState))
  {
    ROS_INFO("Set state");
  }
  else
  {
    ROS_INFO("Problem setting state");
  }
}



ObInfo generateObInfoSimple(const MotionState robot_state)
{
  ObInfo result;

  Range v(0,  0.33);
  Range w(0,  PI/2.f);

  result.v = v.random();
  result.w = w.random();
  result.relative_direction = 0;

  result.x = robot_state.msg_.positions[0];
  result.y = robot_state.msg_.positions[1] + 1;

  result.d = 1;
  
  return result;
}


std::vector<double> getPos()
{
  Range x(0.75, 2.);
  Range y(0.75, 2.);

  double ob_x = x.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_x *= 10;
  ob_x = round(ob_x);
  ob_x /= 10;

  double ob_y = y.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_y *= 10;
  ob_y = round(ob_y);
  ob_y /= 10;


  std::vector<double> result;
  result.push_back(ob_x);
  result.push_back(ob_y);
  return result;
}


// Generate an obstacle with all of its data filled in
ObInfoExt generateObInfoGridExt(const MotionState robot_state)
{
  ObInfoExt result;

  Range x(0.5, 2.0);
  Range y(0.5, 2.0);

  double ob_x = x.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_x *= 10;
  ob_x = round(ob_x);
  ob_x /= 10;

  double ob_y = y.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_y *= 10;
  ob_y = round(ob_y);
  ob_y /= 10;

  // Relative direction depends on x and y
  result.relative_direction = utility.displaceAngle(atan( ob_y / ob_x ), PI);

  
  // Set speeds
  Range v(0, 0.5);
  Range w(-PI/2.f, PI/2.f);

  result.x = ob_x;
  result.y = ob_y;

  result.v_i  = v.random();
  result.v_f  = v.random();
  result.w    = w.random();


  // Set durations
  // ***
  Range durs(0,5);
  result.d_s = ros::Duration(durs.random());
  //result.d_vi = ros::Duration(durs.random());
  //result.d_vf = ros::Duration(durs.random());

  //ROS_INFO("Test case v_i: %f v_f: %f d_s: %f d_vi: %f d_vf: %f", result.v_i, result.v_f, result.d_s.toSec(), result.d_vi.toSec(), result.d_vf.toSec());
  
  
  return result;
}



ObInfo generateObInfoGrid(const MotionState robot_state)
{
  ObInfo result;

  Range x(0.75, 2.);
  Range y(0.75, 2.);

  double ob_x = x.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_x *= 10;
  ob_x = round(ob_x);
  ob_x /= 10;

  double ob_y = y.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_y *= 10;
  ob_y = round(ob_y);
  ob_y /= 10;


  Range v(0, 0.5);
  Range w(0, PI/2.f);

  result.x = ob_x;
  result.y = ob_y;
  result.v = v.random();
  result.w = w.random();
  
  result.relative_direction = utility.displaceAngle(atan( ob_y / ob_x ), PI);
  
  return result;
}

ObInfo generateObInfo(const MotionState robot_state)
{
  ObInfo result;

  Range dist(0, 3.5);

  Range v       (0, 0.5);
  Range w       (0      ,  PI/2.f);
  Range rela_dir(PI/6.f ,  5.f*PI/6.f);
  
  
  result.v                  = v.random();
  result.w                  = w.random();
  result.relative_direction = rela_dir.random();
  
  
  result.d    = dist.random();
  double ob_x = robot_state.msg_.positions[0] + result.d*cos(result.relative_direction);
  double ob_y = robot_state.msg_.positions[1] + result.d*sin(result.relative_direction);

  if(ob_x > 3.5)
    ob_x = 3.5;
  else if(ob_x < 0)
    ob_x = 0.;
  if(ob_y > 3.5)
    ob_y = 3.5;
  else if(ob_y < 0)
    ob_y = 0.;

  result.x = ob_x;
  result.y = ob_y;

  result.faster = result.v > sqrt(pow(robot_state.msg_.velocities[0],2) + pow(robot_state.msg_.velocities[1],2));

  return result;
}


/*
 * p_x and p_y are the position values,                 range: [0, 3.5]
 * v_mag is the magnitude of the linear velocity,       range: [0, 0.25]
 * v_direction is the direction of the linear velocity, range: [0, pi]
 * w is the angular velocity (not a vector),            range: [-pi/4, pi/4]
 */
const ramp_msgs::Obstacle buildObstacleMsg(const double& p_x, const double& p_y, const double& v_mag, const double& v_direction, const double& w)
{
  //ROS_INFO("p_x: %f p_y: %f, v_mag: %f v_direction: %f w: %f", p_x, p_y, v_mag, v_direction, w);

  ramp_msgs::Obstacle result;

  // odom_msg describes the obstacle's position and velocity
  nav_msgs::Odometry odom_msg;

  // Set the positions
  result.ob_ms.positions.push_back(p_x);
  result.ob_ms.positions.push_back(p_y);
  result.ob_ms.positions.push_back(v_direction);
  
  // For linear velocity, calculate x and y components
  double v_x = v_mag*cos(v_direction);
  double v_y = v_mag*sin(v_direction);

  // Set velocities
  result.ob_ms.velocities.push_back(v_mag*cos(v_direction));
  result.ob_ms.velocities.push_back(v_mag*sin(v_direction));
  result.ob_ms.velocities.push_back(w);

  // Add in obstacle radius
  result.cirGroup.fitCir.radius = 0.21;
  result.cirGroup.packedCirs.push_back(result.cirGroup.fitCir);

  return result;
}



TestCase generateTestCase(const MotionState robot_state, double inner_r, double outter_r, int num_obs)
{
  TestCase result;

  Range history(0,20);

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    //ObInfo temp = generateObInfo(robot_state);
    ObInfo temp = generateObInfoSimple(robot_state);

    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v, temp.relative_direction, temp.w);
    
    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);

    result.history = history.random();
  }

  return result;
}

/*
 * Does NOT generate an ABTC
 * Put that in later on once single test case is working well
 */
TestCaseTwo generateTestCase(const MotionState robot_state, int num_obs)
{
  TestCaseTwo result;

  ROS_INFO("In generateTestCase");
  ROS_INFO("num_obs: %i", num_obs);

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    ObInfo temp = generateObInfoGrid(robot_state);

    if(i == 1)
    {
      // Get position distance from other obstacle
      std::vector<double> one, two;
      one.push_back(result.obs[0].x);
      one.push_back(result.obs[0].y);
      two.push_back(temp.x);
      two.push_back(temp.y);
      double dist = utility.positionDistance(one, two);
      while(dist < 0.2) {
        ROS_INFO("one: (%f, %f) temp: (%f, %f)", one[0], one[1], two[0], two[1]);
        
        temp = generateObInfoGrid(robot_state);
        two[0] = temp.x;
        two[1] = temp.y;
        dist = utility.positionDistance(one, two);
      }

    }
    else if (i == 2)
    {
      // Get position distance from other two obstacles
      std::vector<double> one, two, three;
      one.push_back(result.obs[0].x);
      one.push_back(result.obs[0].y);
      two.push_back(result.obs[1].x);
      two.push_back(result.obs[1].y);
      three.push_back(temp.x);
      three.push_back(temp.y);
      double dist1 = utility.positionDistance(one, three);
      double dist2 = utility.positionDistance(two, three);
      while(dist1 < 0.2 || dist2 < 0.2)
      {
        
        ROS_INFO("one: (%f, %f) two:(%f, %f) temp: (%f, %f)", one[0], one[1], two[0], two[1], three[0], three[1]);
        temp = generateObInfoGrid(robot_state);
        three[0] = temp.x;
        three[1] = temp.y;
        dist1 = utility.positionDistance(one, three);
        dist2 = utility.positionDistance(two, three);
      }
    }

    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v, temp.relative_direction, temp.w);
    
    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);
    ROS_INFO("result.obs.size(): %i", (int)result.obs.size());
    ROS_INFO("result.ob_list.obstacles.size(): %i", (int)result.ob_list.obstacles.size());
  }

  return result;
}





/*
 * Does NOT generate an ABTC
 * Put that in later on once single test case is working well
 */
TestCaseExt generateTestCaseExt(const MotionState robot_state, int i_test, int num_obs, bool useFile, double& d_states)
{
  TestCaseExt result;

  //ROS_INFO("In generateTestCaseExt");
  //ROS_INFO("num_obs: %i", num_obs);
  
  double obInitD = 0.75;

  // Get the random duration for state changes
  Range r(2, 5);
  result.d_states = ros::Duration( r.random() );
  ROS_INFO("d_states: %f", result.d_states.toSec());
    
  // Here, use file to get test info
  if(useFile)
  {
    std::string filename = "/home/sterlingm/ros_workspace/src/ramp/data/system-level-testing/ext/0-1-3/testInfo.txt";
    std::ifstream f_info;
    f_info.open(filename, std::ios::in | std::ios::binary); 

    // First, skip ahead to the current test case lines
    for(int i=0;i<i_test;i++)
    {
      std::string temp;

      // Read 4 lines per test
      for(int j=0;j<4;j++)
      {
        std::getline(f_info, temp, '\n');
      }
    }

    for(int i=0;i<num_obs;i++)
    {
      std::string obStr;
      std::getline(f_info, obStr, '\n');
      ROS_INFO("obStr: %s", obStr.c_str());

      ObInfoExt ob;

      // Tokenize the string
      char* token = strtok((char*)obStr.c_str(), " "); 
      int t=0;
      while(token != NULL)
      {
        ROS_INFO("token: %s", token);
        double v = atof(token);
        switch(t)
        {
          case 0:
            ob.x = v;
            break;
          case 1:
            ob.y = v;
            break;
          case 2:
            ob.relative_direction = v;
            break;
          case 3:
            ob.d = v;
            break;
          case 4:
            ob.v_i = v;
            break;
          case 5:
            ob.v_f = v;
            break;
          case 6:
            ob.w = v;
            break;
          case 7:
            ob.d_s = ros::Duration(v);
            break;
        }

        token = strtok(NULL, " ");
        t++;
      } // end while tokens
    
      // Make obstacle msg
      ob.msg = buildObstacleMsg(ob.x, ob.y, ob.v_i, ob.relative_direction, ob.w);
      result.obs.push_back(ob);
      result.ob_list.obstacles.push_back(ob.msg);
    } // end for each obstacle
    
    // get d_states value for test case 
    std::string dstatesStr;
    std::getline(f_info, dstatesStr, '\n');
    d_states = atof((char*)dstatesStr.c_str());

    f_info.close();
  } // end if useFile

  else
  {

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    ObInfoExt temp = generateObInfoGridExt(robot_state);

    if(i == 1)
    {
      // Get position distance from other obstacle
      std::vector<double> one, two;
      one.push_back(result.obs[0].x);
      one.push_back(result.obs[0].y);
      two.push_back(temp.x);
      two.push_back(temp.y);
      double dist = utility.positionDistance(one, two);
      while(dist < obInitD) 
      {
        //ROS_INFO("one: (%f, %f) temp: (%f, %f)", one[0], one[1], two[0], two[1]);
        
        temp = generateObInfoGridExt(robot_state);
        two[0] = temp.x;
        two[1] = temp.y;
        dist = utility.positionDistance(one, two);
      }

    }
    else if (i == 2)
    {
      // Get position distance from other two obstacles
      std::vector<double> one, two, three;
      one.push_back(result.obs[0].x);
      one.push_back(result.obs[0].y);
      two.push_back(result.obs[1].x);
      two.push_back(result.obs[1].y);
      three.push_back(temp.x);
      three.push_back(temp.y);
      double dist1 = utility.positionDistance(one, three);
      double dist2 = utility.positionDistance(two, three);
      while(dist1 < obInitD || dist2 < obInitD)
      {
        //ROS_INFO("one: (%f, %f) two:(%f, %f) temp: (%f, %f)", one[0], one[1], two[0], two[1], three[0], three[1]);
        temp = generateObInfoGridExt(robot_state);
        three[0] = temp.x;
        three[1] = temp.y;
        dist1 = utility.positionDistance(one, three);
        dist2 = utility.positionDistance(two, three);
      }
    }

    // What is this used for?
    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v_i, temp.relative_direction, temp.w);

    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);
    //ROS_INFO("result.obs.size(): %i", (int)result.obs.size());
    //ROS_INFO("result.ob_list.obstacles.size(): %i", (int)result.ob_list.obstacles.size());
  }
  
  } // end else not using file


  /*
   ********************************************************************
   *    Manually set the initial delay values to control the ABTC! 
   ********************************************************************
   */
  // 0
  double d_initialDelay = 0;
  ROS_INFO("Initial delay: %f", d_initialDelay);
  
  // 1 (1 after previous obstacle)
  Range r_secondDelay(result.d_states.toSec()*0.0, result.d_states.toSec()*0.9);
  double d_secondDelay = r_secondDelay.random();
  ROS_INFO("Second delay: %f total: %f", d_secondDelay, d_initialDelay + d_secondDelay);

  // 3 (2 after previous obstacle)
  Range r_thirdDelay(result.d_states.toSec()*1.0, result.d_states.toSec()*1.9);
  double d_thirdDelay = r_thirdDelay.random();
  ROS_INFO("Third delay: %f total: %f", d_thirdDelay, d_initialDelay + d_secondDelay + d_thirdDelay);
  

  // Set all of them
  result.obs[0].d_s = ros::Duration(d_initialDelay);
  //result.obs[1].d_s = ros::Duration(d_initialDelay + d_secondDelay);
  //result.obs[2].d_s = ros::Duration(d_initialDelay + d_secondDelay + d_thirdDelay);
 

  return result;
}




MotionState getGoal(const MotionState init, const double dim)
{
  //ROS_INFO("getGoal init: %s", init.toString().c_str());

  double r = sqrt( pow(dim,2) * 2 );
  double x = init.msg_.positions[0] + r*cos(PI/4.f);
  double y = init.msg_.positions[1] + r*sin(PI/4.f);

  MotionState result;
  result.msg_.positions.push_back(x);
  result.msg_.positions.push_back(y);
  result.msg_.positions.push_back(init.msg_.positions[2]);

  //ROS_INFO("getGoal result: %s", result.toString().c_str());
  return result;
}



/*
 * Publish obstacle information at 20Hz to simulate sensing cycles
 */
void pubObTrj(const ros::TimerEvent e, TestCaseTwo& tc)
{
  ROS_INFO("In pubObTrj");
  ROS_INFO("tc.t_begin: %f", tc.t_begin.toSec());
  ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());

  ros::Duration d_elapsed = ros::Time::now() - tc.t_begin;
  
  //int index = d_elapsed.toSec() * 10;
  //ROS_INFO("index: %i traj size: %i", index, (int)tc.ob_trjs[0].trajectory.points.size()); 

  for(int i=0;i<tc.ob_trjs.size();i++)
  {
    //ROS_INFO("i: %i ob_delay[%i]: %i", i, i, ob_delay[i]);
    //ROS_INFO("Elapsed time: %f",(ros::Time::now() - tc.t_begin).toSec());
    double d_elap_ob = d_elapsed.toSec() - ob_delay[i];
    int index = d_elap_ob*10;
    //ROS_INFO("d_elap_ob: %f index: %i", d_elap_ob, index);

    if( (ros::Time::now() - tc.t_begin).toSec() > ob_delay[i])
    {
      int temp_index = index >= (tc.ob_trjs[i].trajectory.points.size()-1) ? tc.ob_trjs[i].trajectory.points.size()-1 : 
        index;
        
      trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[i].trajectory.points[temp_index]; 

      // Build new obstacle msg
      ramp_msgs::Obstacle ob;
      if(index >= (tc.ob_trjs[i].trajectory.points.size()-1))
      {
        ob = buildObstacleMsg(p.positions[0], p.positions[1], 0, p.positions[2], 0);
      }
      else
      {
        ob = buildObstacleMsg(p.positions[0], p.positions[1], tc.obs[i].v, p.positions[2], tc.obs[i].w);
      } 

     
      tc.obs[i].msg = ob;
      tc.ob_list.obstacles[i] = ob;
    }
  }


  pub_obs.publish(tc.ob_list);
}




/*
 * Publish obstacle information at 20Hz to simulate sensing cycles
 */
void pubObTrjExt(const ros::TimerEvent e, TestCaseExt& tc)
{
  /*ROS_INFO("In pubObTrjExt");
  ROS_INFO("tc.t_begin: %f", tc.t_begin.toSec());
  ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());*/

  ros::Duration d_elapsed = ros::Time::now() - tc.t_begin;
  
  //ROS_INFO("d_elapsed: %f traj size: %i", d_elapsed.toSec(), tc.ob_trjs[0].trajectory.points.size()); 

  for(int i=0;i<tc.ob_trjs.size();i++)
  {
    //ROS_INFO("i: %i ob_delay[%i]: %i", i, i, ob_delay[i]);
    //ROS_INFO("Elapsed time: %f",(ros::Time::now() - tc.t_begin).toSec());
    double d_elap_ob = d_elapsed.toSec() - tc.obs[i].d_s.toSec();
    int index = d_elap_ob*10;
    //ROS_INFO("d_elap_ob: %f index: %i", d_elap_ob, index);
    //ROS_INFO("d_s.toSec(): %f", tc.obs[i].d_s.toSec());

    if( (ros::Time::now() - tc.t_begin).toSec() > tc.obs[i].d_s.toSec())
    {
      //ROS_INFO("size: %i", tc.ob_trjs[i].trajectory.points.size()-1);
      //ROS_INFO("Publishing ob trj");
      int temp_index = index >= ((int)tc.ob_trjs[i].trajectory.points.size()-1) ? tc.ob_trjs[i].trajectory.points.size()-1 : 
        index;
        
      trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[i].trajectory.points[temp_index]; 
      //ROS_INFO("temp_index: %i p: %s", temp_index, utility.toString(p).c_str());

      // Build new obstacle msg
      ramp_msgs::Obstacle ob;
      if(index >= (tc.ob_trjs[i].trajectory.points.size()-1))
      {
        ob = buildObstacleMsg(p.positions[0], p.positions[1], 0, p.positions[2], 0);
      }
      else
      {
        double s = sqrt( pow(p.velocities[0],2) + pow(p.velocities[1],2) );
        ob = buildObstacleMsg(p.positions[0], p.positions[1], s, p.positions[2], tc.obs[i].w);
      }

      //ROS_INFO("ob: %s", utility.toString(ob.ob_ms).c_str());
      

      tc.obs[i].msg = ob;
      tc.ob_list.obstacles[i] = ob;

      globalTc.obs[i].msg = ob;
      globalTc.ob_list.obstacles[i] = ob;
    }
  }
  
  pub_obs.publish(tc.ob_list);
}





/*
 * Publish obstacle information at 20Hz to simulate sensing cycles
 */
void pubObTrjGazebo(const ros::TimerEvent e, TestCaseExt& tc)
{
  ROS_INFO("In pubObTrjGazebo");
  ROS_INFO("tc.t_begin: %f", tc.t_begin.toSec());
  ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());

  ros::Duration d_elapsed = ros::Time::now() - tc.t_begin;
      
  double dRobotThreshold = 0.5;
  
  for(int i=0;i<tc.ob_trjs.size();i++)
  {
    ROS_INFO("Elapsed time: %f",(ros::Time::now() - tc.t_begin).toSec());
    double d_elap_ob = d_elapsed.toSec() - tc.obs[i].d_s.toSec();
    int index = d_elap_ob*10;

    if( (ros::Time::now() - tc.t_begin).toSec() > tc.obs[i].d_s.toSec())
    {
      ROS_INFO("In if");

      int temp_index = index >= ((int)tc.ob_trjs[i].trajectory.points.size()-1) ? tc.ob_trjs[i].trajectory.points.size()-1 : 
        index;

      if(temp_index - tc.obs[i].last_index > 2)
      {
        temp_index = tc.obs[i].last_index;
      }
        
      trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[i].trajectory.points[temp_index]; 

      globalTc.obs[i].msg.ob_ms.positions = p.positions;

      std::ostringstream name;
      name<<"system_level_obstacle";
      //if(i>0)
      name<<"_"<<i;
      
      
      // Get the position of the obstacle in gazebo
      gazebo_msgs::GetModelState getModelState;
      gazebo_msgs::ModelState modelState; 
      modelState.model_name = name.str();

      getModelState.request.model_name = name.str();
      
      getModelSrv.call(getModelState);

      // Get distance to robot
      double dRobot = sqrt( pow( getModelState.response.pose.position.x - latestUpdate.msg_.positions[0], 2) + pow(getModelState.response.pose.position.y - latestUpdate.msg_.positions[1], 2) );


      if(dRobot > dRobotThreshold)
      {
        // Don't need to build an obstacle msg, need to move in Gazebo
        gazebo_msgs::SetModelState setModelState;
        gazebo_msgs::ModelState modelState; 
        modelState.model_name = name.str();
        modelState.reference_frame = "map";
        modelState.pose.position.x = p.positions[0];
        modelState.pose.position.y = p.positions[1];

        setModelState.request.model_state = modelState;

        if(setModelSrv.call(setModelState))
        {
          ROS_INFO("Set state");
        }
        else
        {
          ROS_INFO("Problem setting state");
        }

        tc.obs[i].last_index = temp_index;
      } // end if greater than distance threshold
    } // end if
  } // end for

  ROS_INFO("Exiting pubObTrjGazebo");
}




void bestTrajCb(const ramp_msgs::RampTrajectory::ConstPtr& msg) 
{
  bestTrajec = *msg;
}

bool checkIfObOnGoal(TestCaseExt tc)
{
  std::vector<double> goal;
  goal.push_back(2);
  goal.push_back(2);
  for(int i=0;i<num_obs;i++)
  {
    trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[i].trajectory.points[tc.ob_trjs[i].trajectory.points.size()-1];

    double dist = utility.positionDistance(p.positions, goal);
    //ROS_INFO("p: %s\ndist: %f", utility.toString(p).c_str(), dist);

    // Do .42 b/c we also need to consider if obstacles is close enough to prevent the robot from getting to the goal w/o collision
    if(dist < 0.42)
    {
      return true;
    }
  }
  
  return false;
}


void imminentCollisionCb(const std_msgs::Bool msg)
{
  ROS_INFO("Gen: In imminentCollisionCb, msg: %s", msg.data ? "True" : "False");
  IC_current = msg.data;
  if(msg.data)
  {
    IC_occur = true;
  }
}

bool checkCollision(ramp_msgs::Obstacle ob)
{
  //ROS_INFO("In checkCollision");
  double dist = utility.positionDistance(ob.ob_ms.positions, latestUpdate.msg_.positions);
  //ROS_INFO("ob: %s\ndist: %f", utility.toString(ob).c_str(), dist);

  return dist < 0.42;
}

void collisionCb(const ros::TimerEvent e, TestCaseExt& tc)
{
  ROS_INFO("In collisionCb");

  for(int i=0;i<num_obs;i++)
  {
      
    if(checkCollision(globalTc.obs[i].msg))
    {
      if(colls[i] == false && IC_current)
      {
        // Check if robot in ic
        icAtColl[i] = true;
      }
      else if(colls[i] == false)
      {
        // Check if robot in ic
        icAtColl[i] = false;
      }
        
      // Set collision to true
      colls[i] = true;
    }
    //else ROS_INFO("No collision");
  } // end for
}

void obstacleCollisonCb(const ros::TimerEvent e)
{
  ROS_INFO("In obstacleCollisonCb");
  double obRadii = 0.21;
  for(int i=0;i<num_obs;i++)
  {
    for(int j=0;j<num_obs;j++)
    {
      if(i != j)
      {
        double d = utility.positionDistance( globalTc.obs[i].msg.ob_ms.positions, globalTc.obs[j].msg.ob_ms.positions);
        if( d <= obRadii*2.0)
        {
          collAmongObs = true; 
        }
      } // end if
    } // end inner for
  } // end outer for
}



void updateCb(const ramp_msgs::MotionState& msg)
{
  //ROS_INFO("In updateCb");
  //ROS_INFO("updateCb msg: %s", utility.toString(msg).c_str());
  latestUpdate = msg;

  tf::Transform T_w_odom;
  T_w_odom.setOrigin(tf::Vector3(0,0,0));
  T_w_odom.setRotation(tf::createQuaternionFromYaw(PI/4.));

  latestUpdate.transformBase(T_w_odom);
  //ROS_INFO("After transform, msg: %s", latestUpdate.toString().c_str());
}







std::string getTestCaseInfo(TestCaseExt tc)
{
  std::ostringstream result;

  for(int i=0;i<tc.obs.size();i++)
  {
    ObInfoExt o = tc.obs[i];
    if(i>0)
      result<<"\n";

    result<<o.x<<" "<<o.y<<" "<<o.relative_direction<<" "<<o.d<<" "<<o.v_i<<" "<<o.v_f<<" "<<o.w<<" "<<o.d_s.toSec();
  }

  result<<"\n"<<tc.d_states;
  

  return result.str();
}



/*
 * Read collAmongObs file in the abtc directory and get the index of all test cases with no obstacle collision
 */ 
std::vector<int> getTestCasesNoObColl(std::string abtcDir)
{
  std::vector<int> result;

  std::string filename = abtcDir + "/collAmongObs.txt";
  std::ifstream f_info;
  f_info.open(filename, std::ios::in | std::ios::binary); 
  if(f_info.is_open() == 0)
  {
    ROS_ERROR("Could not open file: %s", filename.c_str());
  } 
  else
  {
    ROS_INFO("file open good");
  }

  
  int i=0;
  std::string temp;

  while(std::getline(f_info, temp, '\n'))
  {
    ROS_INFO("i: %i temp: %s", i, temp.c_str());
    
    if(std::atoi(temp.c_str()) == 0)
    {
      result.push_back(i);
    }

    i++;
  }

  for(int i=0;i<result.size();i++)
  {
    ROS_INFO("result[%i]: %i", i, result[i]);
  }

  return result;
}



void shutdown(int sigint)
{
  // Set flag signifying that the next test case is not ready
  ros::param::set("/ramp/tc_generated", false);
}




int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  // Set function to run at shutdown
  signal(SIGINT, shutdown);
  
  
  // Load ros parameters and obstacle transforms
  //loadParameters(handle);
  //loadObstacleTF();

  num_obs = 3;

  ros::Rate r(100);

  /*
   * Data to collect
   */
  std::vector<bool>   reached_goal;
  std::vector<bool>   bestTrajec_fe;
  std::vector<double> time_left;
  std::vector<bool>   stuck_in_ic;
  std::vector<bool>   ic_occurred;
  std::vector<bool>   ob_on_goal;
  std::vector<TestCaseTwo> test_cases;
  std::vector<TestCaseExt> test_cases_ext;

  
  ros::Timer ob_trj_timer, checkCollTimer, checkCollAmongObsTimer;
  ob_trj_timer.stop();
  checkCollTimer.stop();
  checkCollAmongObsTimer.stop();
  
  int num_tests = 35;


  // Make an ObstacleList Publisher
  pub_obs = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_resetOdom = handle.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);

  ros::ServiceClient trj_gen = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");

  //gaz
  setModelSrv = handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  getModelSrv = handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  ROS_INFO("Created serviceClient");

  ros::Subscriber sub_bestT = handle.subscribe("bestTrajec", 1, bestTrajCb);
  ros::Subscriber sub_imminent_collision_ = handle.subscribe("imminent_collision", 1, imminentCollisionCb);
  ros::Subscriber sub_update = handle.subscribe("update", 1, updateCb);

  //ros::Timer collTimer = handle.createTimer(collisionCb);
  //ob_trj_timer = handle.createTimer(ros::Duration(1./20.), boost::bind(pubObTrjExt, _1, tc));
 

  std::string path = "/home/sterlingm/ros_workspace/src/ramp/data/system-level-testing/ext/0-1-3/";


  // Open files for data
  std::ofstream f_reached;
  f_reached.open(path + "reached_goal.txt", 
      std::ios::out | std::ios::app | std::ios::binary);
  
  std::ofstream f_feasible;
  f_feasible.open(path + "feasible.txt", std::ios::out 
      | std::ios::app | std::ios::binary);

  std::ofstream f_time_left;
  f_time_left.open(path + "time_left.txt", 
      std::ios::out | std::ios::app | std::ios::binary);
  
  std::ofstream f_ic_stuck;
  f_ic_stuck.open(path + "ic_stuck.txt", std::ios::out 
      | std::ios::app | std::ios::binary);
  
  std::ofstream f_ic_occurred;
  f_ic_occurred.open(path + "ic_occurred.txt", 
      std::ios::out | std::ios::app | std::ios::binary);
  
  std::ofstream f_ob_on_goal;
  f_ob_on_goal.open(path + "ob_on_goal.txt", 
      std::ios::out | std::ios::app | std::ios::binary);
  
  std::ofstream f_colls;
  f_colls.open(path + "colls.txt", 
      std::ios::out | std::ios::app | std::ios::binary);
  
  std::ofstream f_icAtColl;
  f_icAtColl.open(path + "icAtColl.txt", 
      std::ios::out | std::ios::app | std::ios::binary);

  std::ofstream f_testInfo;
  f_testInfo.open(path + "testInfo.txt", 
      std::ios::out | std::ios::app | std::ios::binary);

  std::ofstream f_collAmongObs;
  f_collAmongObs.open(path + "collAmongObs.txt",
      std::ios::out | std::ios::app | std::ios::binary);

  ROS_INFO("Opened files");
  
  // Set flag signifying that the next test case is not ready
  ros::param::set("/ramp/tc_generated", false);


  for(int i=0;i<num_obs;i++)
  {
    colls.push_back(false);
    icAtColl.push_back(false);
  }


  ROS_INFO("Calling getTestCasesNoObColl");
  // Build an index list
  //
  std::vector<int> noObCollTestInd = getTestCasesNoObColl("/home/sterlingm/ros_workspace/src/ramp/data/system-level-testing/ext/0-1-3");


  ros::Duration d_history(1);
  ros::Duration d_test_case_thresh(20);
  for(int i=0;i<num_tests;i++)
  {
    setRobotInitialPos();

    /*
     *
     * Generate a test case
     *
     */
   
    /*
     * Generate the test case
     */

    ABTC abtc;

    /*
     * Create test case where all obstacles stop, move, stop for 1 second each
     */
    for(int i_ob=0;i_ob<num_obs;i_ob++)
    {
      abtc.moving[i_ob]   = 0;
      abtc.moving[i_ob+3] = 1;
      abtc.moving[i_ob+6] = 0;
      abtc.times[i_ob] = 1;
      abtc.times[i_ob+3] = 1;
      abtc.times[i_ob+6] = 1;
    }
    
    /*
     * Get test data for the abtc
     */
    //TestCaseTwo tc = generateTestCase(initial_state, num_obs);
    ramp_msgs::MotionState initial_state;
    double d_states;
    TestCaseExt tc = generateTestCaseExt(initial_state, noObCollTestInd[i], num_obs, true, d_states);
    tc.d_states = ros::Duration(d_states);
    tc.abtc = abtc; 
    ROS_INFO("Done generate test case");

    /*
     * Get trajectories for each obstacle
     */
    ramp_msgs::TrajectorySrv tr_srv;
    for(int j=0;j<tc.obs.size();j++)
    {
      ROS_INFO("Getting trajectory for obstacle %i", j);
      ramp_msgs::TrajectoryRequest tr;

      tf::Transform tf;
      tf.setOrigin( tf::Vector3(0,0,0) );
      tf.setRotation( tf::createQuaternionFromYaw(0) );
      MotionType mt = my_planner.findMotionType(tc.obs[j].msg);

      // Temporarily set the ob speed to the final speed to avoid overshooting
      // the trajec. goal and causing the trajec to 'back up'
      // May still face issues with circular arc trajectories?
      ramp_msgs::Obstacle o = tc.obs[j].msg;

      if(tc.obs[j].v_f > tc.obs[j].v_i)
      {
        o.ob_ms.velocities[0] = (tc.obs[j].v_f*2) * cos(tc.obs[j].relative_direction);
        o.ob_ms.velocities[1] = (tc.obs[j].v_f*2) * sin(tc.obs[j].relative_direction);
      }

      
      // Get the path. Use the ob msg with final speed
      //ramp_msgs::Path p = my_planner.getObstaclePath(tc.obs[i].msg, mt);
      ramp_msgs::Path p = my_planner.getObstaclePath(o, mt);

      // Set 1st point on path velocity back to initial velocity
      p.points[0].motionState.velocities[0] = tc.obs[j].v_i * cos(tc.obs[j].relative_direction);
      p.points[0].motionState.velocities[1] = tc.obs[j].v_i * sin(tc.obs[j].relative_direction);

      // Set tr members
      tr.path = p;
      tr.type = PREDICTION;

      // Set Ext ob model stuff
      tr.sl_traj        = true;
      tr.sl_final_speed = tc.obs[j].v_f;
      //tr.sl_init_dur    = tc.obs[j].d_vi;
      //tr.sl_final_dur   = tc.obs[j].d_vf;
      // Changed to have a static time to be in a state for all obstacles
      tr.sl_init_dur    = tc.d_states;
      tr.sl_final_dur   = tc.d_states;
    
      // Get initial delay from hard-coded vector (for now)
      //tr.sl_init_dur    = tc.obs[j].d_s;

      // Push back request
      tr_srv.request.reqs.push_back(tr);


      // Initialize colls and icAtColls
      // gaz
      //colls[j] = false;
      //icAtColl[j] = false;
    } // end for
    ROS_INFO("Done getting obstacle trajectories");

    // Call trajectory generator
    if(trj_gen.call(tr_srv))
    {
      for(int i=0;i<tr_srv.response.resps.size();i++)
      {
        //ROS_INFO("Ob Traj: %s", utility.toString(tr_srv.response.resps[i].trajectory).c_str());
        tc.ob_trjs.push_back(tr_srv.response.resps[i].trajectory);
      }
    }
    else
    {
      ROS_ERROR("Error in getting obstacle trajectories");
    }

    /*
     * Done getting obstacle trajectories
     */


    ROS_INFO("Setting robot initial pose...");
    setRobotInitialPos();
    d_history.sleep();
    setRobotInitialPos();
    resetRobotOdom();
    ROS_INFO("Done setting robot initial pose...");

 
    /*
     * Get static version of obstacles
     */
    ramp_msgs::ObstacleList obs_stat;
    for(int i=0;i<tc.obs.size();i++)
    {
      obs_stat.obstacles.push_back(getStaticOb(tc.obs[i].msg));
      std::ostringstream name;
      name<<"system_level_obstacle";
      //if(i>0)
      name<<"_"<<i;
      setStaticObGazebo(tc.obs[i].msg, name.str());
    }
    ROS_INFO("Generate: obs_stat.size(): %i", (int)obs_stat.obstacles.size());

    setRobotInitialPos();

    IC_occur = false;

    // Set flag signifying that the next test case is ready
    ros::param::set("/ramp/tc_generated", true);

    ROS_INFO("Generate: Waiting for planner to prepare");
    /*
     * Wait for planner to be ready to start test case
     */
    bool start_tc = false;
    while(!start_tc)
    {
      handle.getParam("/ramp/ready_tc", start_tc);
      r.sleep();
      ros::spinOnce();
    }

    ROS_INFO("Generate: Planner ready, publishing static obstacles");

    // Set static-obs param true
    ROS_INFO("Generate: Setting flag for static obs to true");
    ros::param::set("/ramp/static_obs", true);

    // Publish static obstacles
    // gaz 
    // Don't need to publish static obstacles b/c planner can see them in Gazebo
    //pub_obs.publish(obs_stat);

    // Wait for 1 second
    d_history.sleep();

    // Set static-obs param false
    ros::param::set("/ramp/static_obs", false);
    ROS_INFO("Generate: static_obs flag set to false");

    // Set dy-obs param true
    ROS_INFO("Generate: Setting flag for dy obs to true");
    ros::param::set("/ramp/dy_obs", true);

    // Publish dynamic obstacles
    // gaz
    // Don't need to publish dynamic obstacles at all
    //pub_obs.publish(tc.ob_list);

    tc.t_begin = ros::Time::now();
    globalTc = tc;
    ROS_INFO("Set t_begin and globalTc");

    // Set ob coll variable
    collAmongObs = false;

    // Create timer to continuously publish obstacle information
    // gaz
    ob_trj_timer = handle.createTimer(ros::Duration(1./20.), boost::bind(pubObTrjGazebo, _1, tc));
    //checkCollTimer = handle.createTimer(ros::Duration(1./20.), boost::bind(collisionCb, _1, tc));
    //checkCollAmongObsTimer = handle.createTimer(ros::Duration(1./20.), obstacleCollisonCb);
    ROS_INFO("Generate: Created timers");

    // Set flag signifying that the next test case is not ready
    ros::param::set("/ramp/tc_generated", false);
    
    /*
     * Wait for planner to run for time threshold
     */
    while(start_tc)
    {
      handle.getParam("ramp/ready_tc", start_tc);
      //ROS_INFO("generate_test_case: start_tc: %s", start_tc ? "True" : "False");
      r.sleep();
      ros::spinOnce();
    }
    // Get execution time
    ros::Duration elasped = ros::Time::now() - tc.t_begin;
    
    // Stop publishing dy obs
    ob_trj_timer.stop();
    //checkCollTimer.stop();
    //checkCollAmongObsTimer.stop();

    ROS_INFO("Generate:Test case done, setting flags back to false");

    // Set dy-obs param false
    ros::param::set("/ramp/dy_obs", false);

    // Print out collision info
    for(int c=0;c<colls.size();c++)
    {
      ROS_INFO("colls[%i]: %s", c, colls[c] ? "True" : "False");
      ROS_INFO("icAtColl[%i]: %s", c, icAtColl[c] ? "True" : "False");
    }

    // Set last_index values back to 0
    for(int o=0;o<tc.obs.size();o++)
    {
      tc.obs[i].last_index=0;
    }


    /*
     * Collect data
     */

    ROS_INFO("Results for test case %i", i);

    MotionState goal;
    goal.msg_.positions.push_back(2);
    goal.msg_.positions.push_back(2);
    goal.msg_.positions.push_back(PI/4.f);
    ROS_INFO("latestUpdate_.msg.positions.size(): %i", (int)latestUpdate.msg_.positions.size());
    double dist = utility.positionDistance( goal.msg_.positions, latestUpdate.msg_.positions );
     
    ROS_INFO("dist: %f latestUpdate: %s", dist, latestUpdate.toString().c_str());

    //if(elasped.toSec()+0.01 < d_test_case_thresh.toSec())
    //if(bestTrajec.trajectory.points.size() < 3)
    if(dist < 0.2)
    {
      reached_goal.push_back(true);
      f_reached<<true<<std::endl;
    }
    else
    {
      reached_goal.push_back(false);
      f_reached<<false<<std::endl;
    }

    ROS_INFO("bestTrajec: %s", utility.toString(bestTrajec).c_str());
      

    if(bestTrajec.feasible)
    {
      bestTrajec_fe.push_back(true);
      time_left.push_back( bestTrajec.trajectory.points[ bestTrajec.trajectory.points.size()-1 
      ].time_from_start.toSec());

      f_feasible<<true<<std::endl;
      f_time_left<<bestTrajec.trajectory.points[ bestTrajec.trajectory.points.size()-1 
      ].time_from_start.toSec()<<std::endl;
    }
    else
    {
      bestTrajec_fe.push_back(false);
      f_feasible<<false<<std::endl;
      //time_left.push_back(9999);
    }

    ROS_INFO("IC: %s", IC_current ? "True" : "False");
    
    if(IC_current)
    {
      stuck_in_ic.push_back(true);
      f_ic_stuck<<true<<std::endl;
    }
    else
    {
      stuck_in_ic.push_back(false);
      f_ic_stuck<<false<<std::endl;
    }

    if(IC_occur)
    {
      ic_occurred.push_back(true);
      f_ic_occurred<<true<<std::endl;
    }
    else
    {
      ic_occurred.push_back(false);
      f_ic_occurred<<false<<std::endl;
    }

    if(checkIfObOnGoal(tc))
    {
      ob_on_goal.push_back(true);
      f_ob_on_goal<<true<<std::endl;
    }
    else
    {
      ob_on_goal.push_back(false);
      f_ob_on_goal<<false<<std::endl;
    }


    int numColls = 0;
    int numICAtColl = 0;
    for(int i=0;i<num_obs;i++)
    {
      numColls += colls[i] ? 1 : 0;  
      numICAtColl += icAtColl[i] ? 1 : 0;
    }
    f_colls<<numColls<<std::endl;
    f_icAtColl<<numICAtColl<<std::endl;

    f_testInfo<<getTestCaseInfo(tc)<<std::endl;

    f_collAmongObs<<collAmongObs<<std::endl;

    bestTrajec_at_end.push_back(bestTrajec);
    test_cases_ext.push_back(tc);
    //test_cases.push_back(tc);
    
    ROS_INFO("Completed test %i", i); 
  } // end for each test case

  f_reached.close();
  f_feasible.close();
  f_time_left.close();
  f_ic_stuck.close();
  f_ic_occurred.close();
  f_ob_on_goal.close();
  f_colls.close();
  f_icAtColl.close();
  f_testInfo.close();
  f_collAmongObs.close();
    

  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}
