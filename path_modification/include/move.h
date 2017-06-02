#ifndef MOVE_H
#define MOVE_H
#include "utility.h"
#include "constraints.h"
#include "range.h"

class Move {
  public:
    Move() {}
    Move(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();
    const std::vector<double> getNewPosition(const double value, const double bound, const double theta, const double r, bool x_dim, bool cosine) const;
    double getNewX(double y, double y_bound, double theta, double r, bool min, bool cosine);
    double getNewY(double x, double x_max, double theta, double r, bool cosine);

    ramp_msgs::Path path_;
    double dir_;
    double dist_;
    double r_;

    Utility utility_;
  private:
    Constraints checkConstraints_;
};

#endif
