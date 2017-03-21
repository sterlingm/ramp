#include "circle_filter.h"


CircleFilter::CircleFilter(uint8_t state_size, BFL::Gaussian* prior, BFL::LinearAnalyticConditionalGaussian* sys_pdf, BFL::LinearAnalyticConditionalGaussian* meas_pdf) : BFL::ExtendedKalmanFilter(prior)
{
  state_size_ = state_size;
  sys_model   = new BFL::LinearAnalyticSystemModelGaussianUncertainty(sys_pdf);
  meas_model  = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);
}


CircleFilter::~CircleFilter() 
{
  // delete statements commented out. 
  // The base class destructor cause a seg fault with these statements included
  //delete prior;
  //delete posterior;
}
  
void CircleFilter::update(MatrixWrapper::ColumnVector y)
{
}



void CircleFilter::update(MatrixWrapper::ColumnVector u, MatrixWrapper::ColumnVector y)
{
  ROS_INFO("In CircleFilter::update");
  if(u.rows() != state_size_)
  {
    ROS_ERROR("u.columns: %i state_size_: %i", u.rows(), state_size_);
  }
  else if(y.rows() != state_size_)
  {
    ROS_ERROR("y.columns: %i state_size_: %i", y.rows(), state_size_);
  }
  else
  {
    ROS_INFO("sys_model: %p meas_model: %p", (void*)sys_model, (void*)meas_model); 
    ROS_INFO("u: [%f, %f, %f, %f]", u[0], u[1], u[2], u[3]);
    ROS_INFO("y: [%f, %f, %f, %f]", y[0], y[1], y[2], y[3]);

    // Call ExtendedKalmanFilter Update method
    Update(sys_model, u, meas_model, y);

    posterior = PostGet();
  }
  ROS_INFO("Exiting CircleFilter::update");
}


void CircleFilter::printPosterior() const
{
  if(posterior != 0)
  {
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
    MatrixWrapper::SymmetricMatrix cov = posterior->CovarianceGet();

    // Print the mean
    for(int i=0;i<state_size_;i++)
    {
      ROS_INFO("Mean[%i]: %f", i, mean[i]);
    } 

    ROS_INFO("cov.det: %f", cov.determinant());
  }
  else
  {
    ROS_ERROR("No posterior because Kalman filter has not been run yet");
  }
}
