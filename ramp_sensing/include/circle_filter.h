#ifndef CIRCLE_FILTER_H
#define CIRCLE_FILTER_H
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussian.h"


class CircleFilter : public BFL::ExtendedKalmanFilter
{
  CircleFilter(BFL::Gaussian* prior);
  ~CircleFilter();

};

#endif
