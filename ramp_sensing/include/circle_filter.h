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
    CircleFilter(uint8_t state_size, BFL::Gaussian* prior, BFL::LinearAnalyticConditionalGaussian* sys_pdf, BFL::LinearAnalyticConditionalGaussian* meas_pdf);
    virtual ~CircleFilter();

    void update(MatrixWrapper::ColumnVector y);
    void update(MatrixWrapper::ColumnVector u, MatrixWrapper::ColumnVector y);
    void printPosterior() const;
    
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = 0;

  private:
    uint8_t state_size_;
    BFL::Gaussian* prior = 0;
    BFL::LinearAnalyticSystemModelGaussianUncertainty* sys_model = 0;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model = 0;

};

#endif
