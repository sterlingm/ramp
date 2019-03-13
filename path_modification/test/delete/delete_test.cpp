
#include <gtest/gtest.h>
#include "../include/delete.h"


TEST(TestSuite, delete_perform)
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

  // make path
  ramp_msgs::Path p;
  ramp_msgs::Path e;// empty path
  ramp_msgs::Path l2;// path size = 2(delete should not perform)
  
  //p path
  p.points.push_back(kp1); 
  p.points.push_back(kp2);
  p.points.push_back(kp3);
  //l2 path
  l2.points.push_back(kp1); 
  l2.points.push_back(kp2);
  
  // record size b4 perform 
  size_t p_size = p.points.size();//3
  size_t e_size = e.points.size();//0
  size_t l2_size = l2.points.size();//2

  // create delete object
  Delete d;
  
  //set size for path p after delete
  d.path_ = p;   
  d.perform();
  int d_p_size = d.path_.points.size();
  // set size for path e after delete
  d.path_ = e;
  d.perform();
  int d_e_size = d.path_.points.size();
  // set size for path l2 after delete
  d.path_ = l2;
  d.perform();
  int d_l2_size = d.path_.points.size();

  // Check the size
  EXPECT_EQ(d_p_size, p_size-1);
  EXPECT_EQ(d_e_size, e_size);
  EXPECT_EQ(d_l2_size, l2_size);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
