#ifndef CIRCLE_FILTER_H
#define CIRCLE_FILTER_H
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
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
