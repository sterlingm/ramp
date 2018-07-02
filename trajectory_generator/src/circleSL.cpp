#include "circleSL.h"

CircleSL::CircleSL() 
{
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
  timeCutoff_ = ros::Duration(3.5);
}

CircleSL::~CircleSL() 
{
  if(reflexxesData_.rml != 0) 
  {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }
  if(reflexxesData_.inputParameters) 
  {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  if(reflexxesData_.outputParameters != 0) 
  {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}


const bool CircleSL::finalStateReached() 
{
  //return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
  return ((reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) 
      || (timeFromStart_ >= timeCutoff_));
}

void CircleSL::init(const ramp_msgs::MotionState s, const ros::Duration sl_init_dur, const ros::Duration sl_final_dur, const double sl_final_speed) 
{
  //std::cout<<"\nIn init\n";
  start_ = s;
  //std::cout<<"\nstart: "<<utility_.toString(start_)<<"\n";
  
  //v=wr
  w_ = s.velocities.at(2);
  v_ = sqrt( pow(s.velocities.at(0),2) + pow(s.velocities.at(1),2) );
  r_ = v_ / w_;
  //std::cout<<"\nw: "<<w_<<" v: "<<v_<<" r: "<<r_;


  // Theta = angle to the position vector
  double theta = utility_.findAngleToVector(s.positions);
  //std::cout<<"\ntheta: "<<theta;
  
  // Angle from start to the center of the circle
  double alpha = PI/2 - start_.positions.at(2);
  //std::cout<<"\nalpha: "<<alpha;

  center_.positions.push_back(s.positions.at(0) - r_*cos(alpha));
  center_.positions.push_back(s.positions.at(1) + r_*sin(alpha));
  /*std::cout<<"\nr_*cos(alpha): "<<r_*cos(alpha);
  std::cout<<"\nr_*sin(alpha): "<<r_*sin(alpha);
  std::cout<<"\ncenter: ("<<center_.positions.at(0)<<", "<<center_.positions.at(1)<<")";*/

  // This is always -alpha, why?
  initCircleTheta_ = utility_.findAngleFromAToB(center_.positions, start_.positions);


  d_init_       = sl_init_dur;
  d_final_      = sl_final_dur;
  final_speed_  = sl_final_speed;

  timeCutoff_ = ros::Duration(5);
  initReflexxes();
  //std::cout<<"\nLeaving init\n";
}



void CircleSL::initReflexxes() {
  //std::cout<<"\nIn initReflexxes\n";

  // Initialize Reflexxes variables
  reflexxesData_.rml              = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters  = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );
  

  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = fabs(w_);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]     = fabs(w_);
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = PI*2;

  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = PI*10;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = fabs(w_);

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;

  reflexxesData_.resultValue = 0;
}


void CircleSL::changeReflexxes()
{
  //ROS_INFO("Current velocity (%f,%f), changing to (%f,%f)", reflexxesData_.inputParameters->MaxVelocityVector->VecData[0], reflexxesData_.inputParameters->MaxVelocityVector->VecData[1], v_/r_, 0);

  //ROS_INFO("Before: v: %f w: %f r: %f", v_, w_, r_);
  
  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  // Change v_ and w_?
  v_ = final_speed_;
  w_ = v_/r_;
  //w_ = final_speed_;
  //v_ = w_*r_;
  
  //ROS_INFO("After: v: %f w: %f r: %f", v_, w_, r_);
  
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]     = fabs(w_)+0.00001;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]  = fabs(w_);

  //ROS_INFO("Changed velocity (%f,%f)", reflexxesData_.inputParameters->MaxVelocityVector->VecData[0], reflexxesData_.inputParameters->MaxVelocityVector->VecData[1]);
 

  //ROS_INFO("Position validity: %i", reflexxesData_.inputParameters->CheckForValidity());
  //! RMLVelocityInputParameters::CheckForValidity()).
}


// TODO: Acceleration
const ramp_msgs::MotionState CircleSL::buildMotionState(const ReflexxesData data) {
  ramp_msgs::MotionState result;

  
  //std::cout<<"\ndata.outputParameters->NewPositionVector->VecData[0]: "<<data.outputParameters->NewPositionVector->VecData[0];

  double circleTheta, orientation;
  // Find the orientation around the circle
  if(w_ > 0) 
  {
    //ROS_INFO("In w>0");
    circleTheta = utility_.displaceAngle(initCircleTheta_, 
        data.outputParameters->NewPositionVector->VecData[0]);
    
    orientation = utility_.displaceAngle(start_.positions.at(2),
        data.outputParameters->NewPositionVector->VecData[0]);
  }
  else 
  {
    //ROS_INFO("In w<=0");
    circleTheta = utility_.displaceAngle(initCircleTheta_, 
        -data.outputParameters->NewPositionVector->VecData[0]);
    
    orientation = utility_.displaceAngle(start_.positions.at(2),
        -data.outputParameters->NewPositionVector->VecData[0]);
  }

  //x^2 + y^2 = (w*r)^2
  //x = sqrt( (w*r)^2 - y^2 )
  double x = fabs(r_)*cos(circleTheta);
  double y = fabs(r_)*sin(circleTheta);
  result.positions.push_back(x+center_.positions.at(0));
  result.positions.push_back(y+center_.positions.at(1));
  result.positions.push_back(orientation);

  /*std::cout<<"\ntheta: "<<circleTheta<<" (x,y): ("<<x<<","<<y<<")";
  std::cout<<"\nx+center_.positions.at(0): "<<x+center_.positions.at(0);
  std::cout<<"\ny+center_.positions.at(1): "<<y+center_.positions.at(1);*/

  //ROS_INFO("initCircleTheta_: %f r: %f circleTheta: %f (x,y): (%f,%f)", initCircleTheta_, r_, circleTheta, x+center_.positions.at(0), y+center_.positions.at(1));
  
  
  double phi = data.outputParameters->NewPositionVector->VecData[0];
  double theta = utility_.findAngleToVector(result.positions);

  //ROS_INFO("phi: %f theta: %f", phi, theta);
  
  //double x_dot = v_*cos(phi)*sin(theta);
  //double y_dot = v_*cos(phi)*cos(theta);
  //double x_dot = v_*sin(result.positions[2]);
  //double y_dot = v_*cos(result.positions[2]);
  double x_dot = v_*cos(result.positions[2]);
  double y_dot = v_*sin(result.positions[2]);

  //ROS_INFO("x_dot: %f y_dot: %f", x_dot, y_dot);

  result.velocities.push_back(x_dot);
  result.velocities.push_back(y_dot);
  result.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[0]);

  // TODO: Compute acceleration properly
  result.accelerations.push_back(0);
  result.accelerations.push_back(0);
  result.accelerations.push_back(0);

  result.time = timeFromStart_.toSec();
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  return result;
}


// v = w*r
// sqrt(x^2 + y^2) = theta_dot * r
const ramp_msgs::MotionState CircleSL::spinOnce() {
  // Calling the Reflexxes OTG algorithm
  reflexxesData_.resultValue = 
    reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, 
                                    reflexxesData_.outputParameters, 
                                    reflexxesData_.flags);

  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  ramp_msgs::MotionState result = buildMotionState(reflexxesData_);

  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = 
      *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
      *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
      *reflexxesData_.outputParameters->NewAccelerationVector;

  return result;
}

const std::vector<ramp_msgs::MotionState> CircleSL::generatePoints() 
{
  //std::cout<<"\nIn generatePoints\n";
  std::vector<ramp_msgs::MotionState> result;


  result.push_back(start_);
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  reflexxesData_.resultValue = 0;
  
  int iStop = d_init_.toSec() * 10;
  for(int i=0;i<iStop;i++)
  {
    result.push_back(spinOnce());
  }
  changeReflexxes();

  iStop = d_final_.toSec() * 10; 
  for(int i=0;i<iStop;i++)
  {
    result.push_back(spinOnce());
  }


  /*while(!finalStateReached()) {
    result.push_back(spinOnce());
  }*/

  return result;
}


