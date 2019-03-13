
#include <gtest/gtest.h>
#include "../include/constraints.h"

TEST(TestSuite, perform)
{

  // create knotpoints
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
  
  // create kp for constraint parameter 
  ramp_msgs::KnotPoint kp4;
  kp4.motionState.positions.push_back(4);
  kp4.motionState.positions.push_back(4);
  kp4.motionState.positions.push_back(4);

  // create path p
  ramp_msgs::Path p;
  p.points.push_back(kp1); 
  p.points.push_back(kp2);
  p.points.push_back(kp3);
  
  //create constraints object
  bool value;
  Constraints c;
  //Constraints::validKPForPath c(kp4,p);
  //c.validKPForPath(kp4,p);
  
  
  // test //
  EXPECT_EQ(value, true);
  

}//end test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


