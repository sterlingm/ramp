#include "utility.h"

Utility::Utility() {
  for(unsigned int i=0;i<3;i++) {
    ramp_msgs::Range temp;
    temp.min = (i < 2 ? 0   : -PI);
    temp.min = (i < 2 ? 3.5 :  PI);
    standardRanges_.push_back(temp);
  }
}


const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath:";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n  "<<i<<": (";

    result<<path.points.at(i).positions.at(0);
    for(unsigned int k=1;k<path.points.at(i).positions.size();k++) {
      result<<", "<<path.points.at(i).positions.at(k);
    }
    result<<")";
  }

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::Configuration c) const {
  std::ostringstream result;

  result<<"\n  (";

  result<<c.K.at(0);
  for(unsigned int k=1;k<c.K.size();k++) {
    result<<", "<<c.K.at(k);
  }
  result<<")";

  return result.str();
}
