#include "circle_filter.h"


CircleFilter::CircleFilter(BFL::Gaussian* prior) : BFL::ExtendedKalmanFilter(prior) {}

CircleFilter::~CircleFilter() {}
  


