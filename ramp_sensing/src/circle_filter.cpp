#include "circle_filter.h"


CircleFilter::CircleFilter(BFL::Gaussian* prior) : BFL::ExtendedKalmanFilter(prior) {}

CircleFilter::~CircleFilter() {}
  



void CircleFilter::update(BFL::SystemModel<MatrixWrapper::ColumnVector>* const sys_model, const MatrixWrapper::ColumnVector& u, BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model, const MatrixWrapper::ColumnVector& s)
{
  SysUpdate(sys_model, u);
  ROS_INFO("After SysUpdate");
  MeasUpdate(meas_model, s, u);
  ROS_INFO("After MeasUpdate");
}
