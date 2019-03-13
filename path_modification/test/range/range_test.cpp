
#include <gtest/gtest.h>
#include "../include/range.h"

TEST(TestSuite, randtest)
{

  // make range object // 
  Range myRange(0,1);
  Range zeroRange(0,0);
  Range sameRange(4,4);

  // call random // 
  float num = myRange.random();
  float zeroNum = zeroRange.random();
  float sameNum = sameRange.random();
  
  // check if num between min and max //
  // <1
  EXPECT_LT(num , 1);
  // >0
  EXPECT_GT(num , 0);
  // =0
  EXPECT_EQ(zeroNum , 0);
  // expect same =4 
  EXPECT_EQ(sameNum , 4);



}// end TEST

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
