#include "path.h"

Path::Path() {}


Path::Path(const KnotPoint start, const KnotPoint goal) : start_(start), goal_(goal) {
  all_.push_back(start);
  all_.push_back(goal); 
}

Path::Path(const MotionState start, const MotionState goal) : start_(start), goal_(goal) {
  KnotPoint kp_s(start);
  KnotPoint kp_g(goal);
  all_.push_back(kp_s);
  all_.push_back(kp_g); 
}



Path::Path(const std::vector<KnotPoint> all) {
  start_ = all.at(0);
  goal_ = all.at(all.size()-1);
  

  for(unsigned int i=0;i<all.size();i++) {
    all_.push_back(all.at(i));
  }
}


Path::Path(const std::vector<MotionState> all) {
  for(uint8_t i=0;i<all.size();i++) {
    KnotPoint temp(all.at(i));
    all_.push_back(temp);
  }
}



Path::Path(const ramp_msgs::Path p) {

  KnotPoint s(p.points.at(0));
  start_ = s;

  KnotPoint g(p.points.at(p.points.size()-1));
  goal_ = g;

  for(unsigned int i=0;i<p.points.size();i++) {
    KnotPoint kp(p.points.at(i));
    all_.push_back(kp);
  }
}

Path::~Path() {}


const bool Path::equals(const Path& p) const {

  if(size() != p.size()) {
    return false;
  }

  for(uint8_t i=0;i<size();i++) 
  {
    if(!all_.at(i).equals(p.all_.at(i))) 
    {
      return false;
    }
  }

  return true;
}

const KnotPoint Path::at(const uint8_t i) const {
  return all_.at(i);
}

// TODO: Why am I only offsetting the intermediate knot points?
void Path::offsetPositions(const MotionState diff)
{
  //for(uint8_t i=1;i<all_.size()-1;i++)
  for(uint8_t i=0;i<all_.size()-1;i++)
  {
    all_.at(i).motionState_ = all_.at(i).motionState_.subtractPosition(diff);
  }

  //start_  = all_.at(0);
  //goal_   = all_.at(all_.size()-1);
}

void Path::addBeforeGoal(const KnotPoint kp) {
  if(all_.size() > 0) {
    all_.insert(all_.end()-1, kp);
  }
  else {
    all_.push_back(kp);
  }
}


/** This method inserts the motion state ms into the path at location path_size-1 */
void Path::addBeforeGoal(const MotionState ms) {
  KnotPoint kp(ms);
  addBeforeGoal(kp);
}


void Path::changeStart(const MotionState ms) {
  KnotPoint kp(ms);
  all_.insert(all_.begin(), kp);
  start_ = kp;
}

const unsigned int Path::size() const { return all_.size(); }


const ramp_msgs::Path Path::buildPathMsg() const {
  ramp_msgs::Path result;

  //Push all of the configurations onto the Path msg
  for(unsigned int i=0;i<all_.size();i++) {

    //Build the motion state msg
    ramp_msgs::KnotPoint mp = all_.at(i).buildKnotPointMsg();

    //Push the msg onto K
    result.points.push_back(mp);
  }

  return result;
}

const std::string Path::toString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<all_.size();i++) {
    result<<"\n  "<<i<<": "<<all_.at(i).toString();
  }
  
  return result.str();
}
