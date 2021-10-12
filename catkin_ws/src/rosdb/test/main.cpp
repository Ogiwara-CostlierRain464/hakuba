#include <gtest/gtest.h>
#include <ros/init.h>

using namespace std;

struct Unit: public ::testing::Test{};

TEST_F(Unit, hello){
  EXPECT_EQ(1+1, 2);
}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros_db");
  return RUN_ALL_TESTS();
}
