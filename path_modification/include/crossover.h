#ifndef CROSSOVER_H
#define CROSSOVER_H
#include "ramp_msgs/Path.h"
#include "utility.h"



class Crossover {
  public:
    Crossover() {}
    Crossover(const ramp_msgs::Path p1, const ramp_msgs::Path p2);
	Crossover(const nav_msgs::Path p1, const nav_msgs::Path p2);

    const std::vector<ramp_msgs::Path> perform();
	const std::vector<nav_msgs::Path> navPerform();

    ramp_msgs::Path path1_;
    ramp_msgs::Path path2_;

	nav_msgs::Path navPath1_;
    nav_msgs::Path navPath2_;
  private:

};

#endif
