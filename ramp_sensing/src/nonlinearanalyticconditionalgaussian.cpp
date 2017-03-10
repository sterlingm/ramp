#include "nonlinearanalyticconditionalgaussian.h"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE),
      df(6,6)
  {
    // initialize df matrix
    for (unsigned int i=1; i<=6; i++){
      for (unsigned int j=1; j<=6; j++){
        if (i==j) df(i,j) = 1;
        else df(i,j) = 0;
      }
    }
  }


  NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile(){}

  ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
  {
    ROS_INFO("In ExpectedValueGet");
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    ROS_INFO("state:");
    for(int i=0;i<7;i++)
    {
      ROS_INFO("state[%i]: %f", i, state[i]);
    }
    state(1) += cos(state(6)) * vel(1);
    state(2) += sin(state(6)) * vel(1);
    state(6) += vel(2);
    
    ROS_INFO("state after changing:");
    for(int i=0;i<7;i++)
    {
      ROS_INFO("state[%i]: %f", i, state[i]);
    }

    return state + AdditiveNoiseMuGet();
  }

Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
{
  ROS_INFO("In dfGet");
  if (i==0)//derivative to the first conditional argument (x)
  {
    double vel_trans = ConditionalArgumentGet(1)(1);
    double yaw = ConditionalArgumentGet(0)(6);

    df(1,3)=-vel_trans*sin(yaw); 
    df(2,3)= vel_trans*cos(yaw);

    return df;
  }
  else
  {
  
    if (i >= NumConditionalArgumentsGet())
    {
      cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
      exit(-BFL_ERRMISUSE);
    }
    else
    {
      cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
      exit(-BFL_ERRMISUSE);
    }
  } // end else
}

}//namespace BFL
