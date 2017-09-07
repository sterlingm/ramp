#ifndef MODIFIER_H
#define MODIFIER_H
#include "utility.h"
#include "population.h"
#include "ramp_msgs/ModificationRequest.h"
#include "modification_request_handler.h"

class Modifier {
  public:
    Modifier(const ros::NodeHandle& h, const unsigned int n);
    ~Modifier();

    // Methods
    const std::vector<Path> perform(const Population pop);
    const ramp_msgs::ModificationRequest buildModificationRequest(const Population pop);


    // Data members
    unsigned int num_ops;
    double dir_;

  private:
    const std::string getOperator() const;
    const std::vector<int> getTargets(const std::string op, const Population pop);

    ModificationRequestHandler* h_mod_req_;
    Utility u;
};

#endif
