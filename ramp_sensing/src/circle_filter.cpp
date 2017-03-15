#include "circle_filter.h"


CircleFilter::CircleFilter(uint8_t state_size, BFL::Gaussian* prior, BFL::LinearAnalyticConditionalGaussian* sys_pdf, BFL::LinearAnalyticConditionalGaussian* meas_pdf) : BFL::ExtendedKalmanFilter(prior)
{
  state_size_ = state_size;
  sys_model   = new BFL::LinearAnalyticSystemModelGaussianUncertainty(sys_pdf);
  meas_model  = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf);
}


CircleFilter::~CircleFilter() 
{
  if(prior)
  {
    delete prior;
  }
  if(sys_model)
  {
    delete sys_model;
  }
  if(meas_model)
  {
    delete meas_model;
  }
}
  


void CircleFilter::update(MatrixWrapper::ColumnVector u, MatrixWrapper::ColumnVector y)
{
  ROS_INFO("In CircleFilter::update");
  if(u.rows() != state_size_)
  {
    ROS_ERROR("u.columns: %i state_size_: %i", u.columns(), state_size_);
  }
  else if(y.rows() != state_size_)
  {
    ROS_ERROR("y.columns: %i state_size_: %i", y.columns(), state_size_);
  }
  else
  {
    ROS_INFO("sys_model: %p meas_model: %p", (void*)sys_model, (void*)meas_model); 
    // Call ExtendedKalmanFilter Update method
    Update(sys_model, u, meas_model, y);
  }
  ROS_INFO("Exiting CircleFilter::update");
}
