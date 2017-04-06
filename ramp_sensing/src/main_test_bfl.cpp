#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
#include "ros/ros.h"
#include "utility.h"


#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussian.h"
//using namespace MatrixWrapper; comment out to compile
//using namespace BFL; ambiguation on ramp_msgs::ObstacleList list



/*********************************
 * Variables for BFL
 *********************************/

const int STATE_SIZE=3;

/*
 * Linear System
 */
BFL::LinearAnalyticConditionalGaussian* sys_pdf=0;
BFL::LinearAnalyticSystemModelGaussianUncertainty* sys_model=0;

/*
 * NonLinear System
 */
BFL::NonLinearAnalyticConditionalGaussianMobile* nl_sys_pdf = 0;
BFL::AnalyticSystemModelGaussianUncertainty* nl_sys_model = 0;
double MU_SYSTEM_NOISE_X = 0.0001;
double MU_SYSTEM_NOISE_V = 0.0001;
double MU_SYSTEM_NOISE_A = 0.0001;
double SIGMA_SYSTEM_NOISE_X = 0.0001;
double SIGMA_SYSTEM_NOISE_V = 0.0001;
double SIGMA_SYSTEM_NOISE_A = 0.0001;

/*
 * Measurement model
 */
BFL::LinearAnalyticConditionalGaussian* meas_pdf = 0;
BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model = 0;
double MU_MEAS_NOISE = 0.0001;
double SIGMA_MEAS_NOISE = 0.0001;



/*
 * Prior distribution
 */
BFL::Gaussian* prior = 0;
double PRIOR_MU_X = 1.5;
double PRIOR_MU_V = 0.0;
double PRIOR_MU_A = 0.0;
double PRIOR_COV_X = 0.0001;
double PRIOR_COV_V = 0.0001;
double PRIOR_COV_A = 0.0001;


BFL::Pdf<MatrixWrapper::ColumnVector>* posterior;

/*
 * Filter
 */
BFL::ExtendedKalmanFilter* ekf = 0;



BFL::ExtendedKalmanFilter makeFilter(BFL::Gaussian prior)
{
  BFL::ExtendedKalmanFilter filter(&prior);
  return filter;
}

void init_measurement_model()
{
  ROS_INFO("In init_measurement_model");

  //  z_k+1 = H_x_k+1
  MatrixWrapper::Matrix H(1,1);
  H(1,1) = 1;

  ROS_INFO("Setting mu");

  BFL::ColumnVector meas_noise_mu(1);
  meas_noise_mu(1) = MU_MEAS_NOISE;
  //meas_noise_mu(2) = MU_MEAS_NOISE;
  //meas_noise_mu(3) = MU_MEAS_NOISE;

  ROS_INFO("Setting cov");

  MatrixWrapper::SymmetricMatrix meas_noise_cov(1,1);
  meas_noise_cov(1,1) = SIGMA_MEAS_NOISE;
  //meas_noise_cov(2,2) = SIGMA_MEAS_NOISE;
  //meas_noise_cov(3,3) = SIGMA_MEAS_NOISE;

  ROS_INFO("Setting measurement_uncertainty");

  // Make the Gaussian
  BFL::Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);

  ROS_INFO("Setting meas_pdf");

  // Make the pdf
  meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty);

  ROS_INFO("Setting meas_model");

  // Make model
  meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);
}

void init_ekf()
{
  ROS_INFO("In init_ekf");

  ekf = new BFL::ExtendedKalmanFilter(prior);
}

void init_prior_model()
{
  ROS_INFO("In init_prior_model");

  // Build prior distribution
  BFL::ColumnVector prior_mu(STATE_SIZE);
  prior_mu(1) = PRIOR_MU_X;
  prior_mu(2) = PRIOR_MU_V;
  prior_mu(3) = PRIOR_MU_A;

  MatrixWrapper::SymmetricMatrix prior_cov(STATE_SIZE);
  prior_cov(1,1) = PRIOR_COV_X;
  prior_cov(2,2) = PRIOR_COV_V;
  prior_cov(3,3) = PRIOR_COV_A;

  prior = new BFL::Gaussian(prior_mu, prior_cov);
}


/*
 * Initialize the nonlinear system model
 */
void init_nonlinear_system_model()
{
  ROS_INFO("In init_nonlinear_system_model");

  MatrixWrapper::ColumnVector sys_noise_Mu(STATE_SIZE);
  sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
  sys_noise_Mu(2) = MU_SYSTEM_NOISE_V;
  sys_noise_Mu(3) = MU_SYSTEM_NOISE_A;


  MatrixWrapper::SymmetricMatrix sys_noise_Cov(STATE_SIZE);
  sys_noise_Cov = 0.0;
  sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
  sys_noise_Cov(1,2) = 0.0;
  sys_noise_Cov(1,3) = 0.0;
  sys_noise_Cov(2,1) = SIGMA_SYSTEM_NOISE_V;
  sys_noise_Cov(2,2) = 0.0;
  sys_noise_Cov(2,3) = 0.0;
  sys_noise_Cov(3,1) = SIGMA_SYSTEM_NOISE_A;
  sys_noise_Cov(3,2) = 0.0;
  sys_noise_Cov(3,3) = 0.0;
    

  BFL::Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

  nl_sys_pdf = new BFL::NonLinearAnalyticConditionalGaussianMobile(system_Uncertainty);
  
  nl_sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(nl_sys_pdf);
}


void update(BFL::SystemModel<MatrixWrapper::ColumnVector>* const sys_model, const MatrixWrapper::ColumnVector& u, BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model, const MatrixWrapper::ColumnVector& y)
{
  ROS_INFO("u: (%f,%f,%f) y: (%f, %f,%f)", u[0], u[1], u[2], y[0], y[1], y[2]);
  
  if(!ekf->Update(nl_sys_model, u, meas_model, y))
  //if(!ekf->Update(meas_model, y))
  {
    ROS_INFO("Problem updating filter!");
  }
  
  posterior = ekf->PostGet(); 
}


void printPosterior()
{
  MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
  MatrixWrapper::SymmetricMatrix cov = posterior->CovarianceGet();

  // Print the mean
  //ROS_INFO("Mean: (%f, %f, %f)", mean[0], mean[1], mean[2]);
  for(int i=0;i<STATE_SIZE;i++)
  {
    ROS_INFO("Mean[%i]: %f", i, mean[i]);
  } 

  ROS_INFO("cov.det: %f", cov.determinant());

  // Print the covariance
  for(int i=0;i<cov.rows();i++)
  {
    for(int j=0;j<cov.columns();j++)
    {
      ROS_INFO("cov[%i][%i]: %f", i, j, cov(i+1,j+1));
    }
  }
}



int main(int argc, char** argv) 
{
  ros::init(argc, argv, "test_bfl");
  ros::NodeHandle handle;
  

  // Initialize the Kalman Filter
  init_nonlinear_system_model();
  init_measurement_model();
  init_prior_model();
  init_ekf();

  double measurements[] = {1.5, 1.65, 1.32, 1.45, 1.24};
  std::vector<double> measurements_vec(measurements, measurements + sizeof(measurements) / sizeof(measurements[0]));

  // Iterate through list of measurements
  for(int i=0;i<measurements_vec.size();i++)
  {
    ROS_INFO("Time %i", i);
    ROS_INFO("measurements[%i]: %f", i, measurements[i]);
    
    MatrixWrapper::ColumnVector u(STATE_SIZE);
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;

    MatrixWrapper::ColumnVector y(1);
    y[0] = measurements[i];
    y[1] = 0;
    y[2] = 0;

    update(nl_sys_model, u, meas_model, y);
    printPosterior();
  }
  


  std::cout<<"\nSpinning\n";

  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();

  std::cout<<"\nExiting Normally\n";
  return 0;
}
