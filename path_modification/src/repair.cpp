#include "repair.h"
#include <swri_profiler/profiler.h>



Repair::Repair(const ramp_msgs::Path p) : path_(p) {}

/*
 * Return vector: 
 *  if x_dim, then [x, y]
 *  else, then [y, x]
 */
const std::vector<double> Repair::getNewPosition(const double value, const double other_value, const double bound, const double theta, const double r, bool x_dim) const
{
  SWRI_PROFILE("PathMod-Repair-Perform");
  /*ROS_INFO("In Repair::getNewPosition");
  ROS_INFO("value: %f other_value: %f", value, other_value);
  ROS_INFO("bound: %f theta: %f r: %f x_dim: %s", bound, theta, r, x_dim ? "True" : "False");*/
  
  double p_prime = bound;
  double delta_p = x_dim ? fabs(bound - path_.points[0].motionState.positions[0]) : fabs(bound - path_.points[0].motionState.positions[1]);

  double denom = x_dim ? cos(theta) : sin(theta);
  double r_prime = fabs(delta_p / denom);
  double other_prime = sqrt( pow(r_prime, 2) - pow(delta_p, 2) );

  //ROS_INFO("p_prime: %f delta_p: %f denom: %f r_prime: %f other_prime: %f", p_prime, delta_p, denom, r_prime, other_prime);

  // Calculate the new coordinates
  double p_new = x_dim ? r_prime * cos(theta) : r_prime * sin(theta);
  p_new = bound;
  double other_new = x_dim ? r_prime * sin(theta) : r_prime * cos(theta);

  //ROS_INFO("other_new: %f", other_new);
  other_new += x_dim ? path_.points[0].motionState.positions[1] : path_.points[0].motionState.positions[0];
  //ROS_INFO("p_new: %f other_new: %f", p_new, other_new);

  std::vector<double> result;
  result.push_back(p_new);
  result.push_back(other_new);
  return result;
}


const ramp_msgs::Path Repair::perform() 
{
  /*ROS_INFO("dir_: %f", dir_);
  ROS_INFO("dist_: %f", dist_);
  ROS_INFO("r: %f", r_);
  ROS_INFO("Before: %s", utility_.toString(path_).c_str());*/

  /*
   * Declare the variables we want to get
   */
  double x, y, theta, dist;
  
  /*
   * Get the variables we will use in this modification
   */  
  // Distance we will use from the current position
  Range dist_range(dist_+r_, 2*(dist_+r_));
  dist = dist_range.random();
  //ROS_INFO("dist: %f", dist);


  // Collision radius, ob radius + robot radius
  double coll_cir_rad = r_+0.275;
  //ROS_INFO("coll_cir_rad: %f", coll_cir_rad);

  /* Current position
  Range dist_range(r_, 3*r_);
  dist = dist_range.random();
  ROS_INFO("dist: %f", dist);

   * Get the new point!
   */

  // Check if we are inside of this collision circle
  // If so, then we use a different strategy by
  // only generating points behind the robot
  if(dist_ - coll_cir_rad < 0.1)
  {
    //ROS_INFO("In if, robot is on edge of collision circle");

    double theta_a = utility_.displaceAngle(dir_, PI/2.);
    double theta_b = utility_.displaceAngle(dir_, -PI/2.);
    //ROS_INFO("theta_a: %f theta_b: %f", theta_a, theta_b);
    
    double diff_theta = utility_.findDistanceBetweenAngles(theta_a, theta_b);
    //ROS_INFO("diff_theta: %f", diff_theta);
    
    Range range_theta(0, PI);
    double theta_displacement = range_theta.random();
    theta = utility_.displaceAngle(theta_a, theta_displacement);

    //ROS_INFO("theta_displacement: %f theta: %f", theta_displacement, theta);

    // Create point
    x = path_.points[0].motionState.positions[0] + cos(theta) * dist;
    y = path_.points[0].motionState.positions[1] + sin(theta) * dist;

    //ROS_INFO("New point: (%f, %f)", x, y);
  }

  // Else if we are significantly far from the collision circle
  else
  {
    //ROS_INFO("In else, robot is not on edge of collision circle");
    
    // cir_theta = opposite angle between the robot and obstacle
    double cir_theta = utility_.displaceAngle(dir_, PI);

    // Displace cir_theta by 45 degrees in each direction to get the points on the circle 
    // that are the widest angles on the circle in the robot's direction
    double cir_displace_pos = utility_.displaceAngle(cir_theta, PI/4.);
    double cir_displace_neg = utility_.displaceAngle(cir_theta, -PI/4.);

    //ROS_INFO("cir_theta: %f cir_displace_pos: %f cir_displace_neg: %f", cir_theta, cir_displace_pos, cir_displace_neg);

    // Store robot's point and obstacle's point
    double robo_x = path_.points[0].motionState.positions[0];
    double robo_y = path_.points[0].motionState.positions[1];
    double ob_x   = robo_x + dist_*cos(dir_);
    double ob_y   = robo_y + dist_*sin(dir_);

    //ROS_INFO("robo pos: (%f, %f) ob pos: (%f, %f)", robo_x, robo_y, ob_x, ob_y);
    //ROS_INFO("ob_x: %f r_: %f cir_displace_pos: %f cos(%f) r_*cos(%f): %f +%f=%f", ob_x, r_, cir_displace_pos, cir_displace_pos, cir_displace_pos, r_*cos(cir_displace_pos), ob_x, ob_x+r_*cos(cir_displace_pos));
    
    /*
     * Find the points on the collision circle in the direction of
     * the widest angles from the circle's center
     */
    std::vector<double> pos, neg;
    
    // (pos_x,pos_y) is the point on the collision circle in the direction of cir_displace_pos
    double pos_x = ob_x + coll_cir_rad*cos(cir_displace_pos);
    double pos_y = ob_y + coll_cir_rad*sin(cir_displace_pos);
    pos.push_back(pos_x);
    pos.push_back(pos_y);

    // (neg_x,neg_y) is the point on the collision circle in the direction of cir_displace_neg
    double neg_x = ob_x + r_*cos(cir_displace_neg);
    double neg_y = ob_y + r_*sin(cir_displace_neg);
    neg.push_back(neg_x);
    neg.push_back(neg_y);

    //ROS_INFO("pos: (%f, %f) neg: (%f, %f)", pos_x, pos_y, neg_x, neg_y);

    /*
     * Find the angles from the robot to the "widest points" on the collision circle
     */
    double pos_theta = utility_.findAngleFromAToB(path_.points[0].motionState.positions, pos);
    double neg_theta = utility_.findAngleFromAToB(path_.points[0].motionState.positions, neg);

    // Find the difference between these angles
    double theta_diff = utility_.findDistanceBetweenAngles(pos_theta, neg_theta);

    //ROS_INFO("pos_theta: %f neg_theta: %f theta_diff: %f", pos_theta, neg_theta, theta_diff);

    /*
     * Displace each angle by 90 degrees to get the angles that are
     * wider than these angles, but not behind the robot
     */
    double theta_a = utility_.displaceAngle(dir_, PI/2.);
    double theta_b = utility_.displaceAngle(dir_, -PI/2.);
    //ROS_INFO("theta_a: %f theta_b: %f", theta_a, theta_b);

    // Create the ranges to generate a random angle
    double diff_pos = utility_.findDistanceBetweenAngles(pos_theta, theta_b);
    double diff_neg = utility_.findDistanceBetweenAngles(neg_theta, theta_a);
    Range dir_range_pos(0, diff_pos);
    Range dir_range_neg(0, diff_neg);
    //ROS_INFO("Positive range: [%f, %f]:", dir_range_pos.msg_.min, dir_range_pos.msg_.max);
    //ROS_INFO("Negative range: [%f, %f]:", dir_range_neg.msg_.min, dir_range_neg.msg_.max);

    /*
     * Displace theta to get right-triangle angle
     * Coin-flip for using one range or the other
     */
    if(rand() % 2 == 0)
    {
      //ROS_INFO("Displacing in positive range");
      double theta_displacement = dir_range_pos.random();
      theta = utility_.displaceAngle(pos_theta, theta_displacement);
      //ROS_INFO("theta_displacement: %f theta: %f", theta_displacement, theta);
    }
    else
    {
      //ROS_INFO("Displacing in negative range");
      double theta_displacement = dir_range_neg.random();
      theta = utility_.displaceAngle(neg_theta, -theta_displacement);
      //ROS_INFO("theta_displacement: %f theta: %f", theta_displacement, theta);
    }


    /*
     * Create point
     */ 
    x = path_.points[0].motionState.positions[0] + (cos(theta) * dist);
    y = path_.points[0].motionState.positions[1] + (sin(theta) * dist);

    //ROS_INFO("New point: (%f, %f)", x, y);
  } // end else
    
  /*
   * Check the bounds
   */
  // x dimension
  if(x < ranges_[0].min)
  {
    std::vector<double> pos = getNewPosition(x, y, ranges_[0].min, theta, dist, true);
    x = pos[0];
    y = pos[1];
  }
  else if(x > ranges_[0].max)
  {
    std::vector<double> pos = getNewPosition(x, y, ranges_[0].max, theta, dist, true);
    x = pos[0];
    y = pos[1];
  }
  
  // y dimension
  if(y < ranges_[1].min)
  {
    std::vector<double> pos = getNewPosition(y, x, ranges_[1].min, theta, dist, false);
    x = pos[1];
    y = pos[0];
  }
  else if(y > ranges_[1].max)
  {
    std::vector<double> pos = getNewPosition(y, x, ranges_[1].max, theta, dist, false);
    x = pos[1];
    y = pos[0];
  }
  
  /*
   * Create KnotPoint and modify the path
   */ 
  ramp_msgs::KnotPoint kp;
  kp.motionState.positions.push_back(x);
  kp.motionState.positions.push_back(y);
  kp.motionState.positions.push_back(theta);

  // Replace
  if(path_.points.size() > 2)
  {
    path_.points.at(1) = kp;
  }
  // Insert 
  else if(path_.points.size() == 2)
  {
    path_.points.insert(path_.points.begin()+1, kp);
  }
  else
  {
    ROS_ERROR("Path has less than 2 points, cannot apply a modification operator");
  }
  
  //ROS_INFO("After: %s", utility_.toString(path_).c_str()); 
  return path_;
}
