#ifndef MODIFIER_H
#define MODIFIER_H
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/NavModificationRequest.h"
#include "insert.h"
#include "delete.h"
#include "change.h"
#include "crossover.h"
#include "swap.h"
#include "repair.h"


class Modifier {
  public:

    Modifier(ramp_msgs::ModificationRequest::Request& req);
    Modifier(ramp_msgs::ModificationRequest::Request& req, std::vector<ramp_msgs::Range> ranges);
    Modifier(ramp_msgs::NavModificationRequest::Request& req, std::vector<ramp_msgs::Range> ranges);
    ~Modifier() {}
    
    Insert in_;
    Delete del_;
    Change chg_;
    Crossover cross_;
    Swap swap_; 
    Repair repair_;
    
    std::vector<ramp_msgs::Range> ranges_;

    ramp_msgs::ModificationRequest::Request mod_req;
    ramp_msgs::NavModificationRequest::Request navMod_req;
    const std::vector<ramp_msgs::Path> perform();
    const std::vector<nav_msgs::Path> navPerform();

    Utility u;
};

#endif
