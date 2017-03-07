#ifndef CIRCLE_FILTER_H
#define CIRCLE_FILTER_H
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussian.h"
#include <ros/console.h>


class CircleFilter : public BFL::ExtendedKalmanFilter
{
  public:
    CircleFilter(BFL::Gaussian* prior);
    ~CircleFilter();

    void update(BFL::SystemModel<MatrixWrapper::ColumnVector>* const sys_model, const MatrixWrapper::ColumnVector& u, BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model, const MatrixWrapper::ColumnVector& s);
};

#endif
