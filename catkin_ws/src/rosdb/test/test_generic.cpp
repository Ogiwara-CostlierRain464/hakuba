#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

using namespace std;
namespace ser = ros::serialization;

struct TestGeneric: public ::testing::Test{};

TEST_F(TestGeneric, topic){
  geometry_msgs::Point point;
}