#include "nonlinearanalyticconditionalgaussian.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianOdo::NonLinearAnalyticConditionalGaussianOdo(const Gaussian& additiveNoise)
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


  NonLinearAnalyticConditionalGaussianOdo::~NonLinearAnalyticConditionalGaussianOdo(){}

  ColumnVector NonLinearAnalyticConditionalGaussianOdo::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += cos(state(6)) * vel(1);
    state(2) += sin(state(6)) * vel(1);
    state(6) += vel(2);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianOdo::dfGet(unsigned int i) const
  {
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
  else{
    cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
    exit(-BFL_ERRMISUSE);
  }
      }
  }

}//namespace BFL
