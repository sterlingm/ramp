
#include <gtest/gtest.h>
#include "../include/range.h"

TEST(TestSuite, randtest_min)
{

  // make range object // 
  Range myrange(0,1);

  // call random // 
  float num = myrange.random();
  
  // check if num between min and max //
  EXPECT_GT(num , 0);



}// end TEST

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
