
#include <gtest/gtest.h>
#include "../include/repair.h"


TEST(TestSuite, repair_perform)
{

  // make kp
  ramp_msgs::KnotPoint kp1;
  kp1.motionState.positions.push_back(1);
  kp1.motionState.positions.push_back(1);
  kp1.motionState.positions.push_back(1);

  ramp_msgs::KnotPoint kp2;
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(2);

  ramp_msgs::KnotPoint kp3;
  kp3.motionState.positions.push_back(3);
  kp3.motionState.positions.push_back(3);
  kp3.motionState.positions.push_back(3);

  //make path
  ramp_msgs::Path p;
  p.points.push_back(kp1); 
  p.points.push_back(kp2);
  p.points.push_back(kp3);

  // set size before perform
  size_t p_size = p.points.size();

  // create repair object
  Repair r;
  r.path_ = p;   
  r.perform();


  // Check the size
  EXPECT_EQ(r.path_.points.size(), p_size);
}// end test


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
