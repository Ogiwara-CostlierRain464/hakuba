#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>
#include "../src/generic_database.h"

using namespace std;
using namespace topic_tools;
namespace ser = ros::serialization;

struct TestGeneric: public ::testing::Test{};

TEST_F(TestGeneric, topic){
  ros::NodeHandle nh_;
  {
    GenericDB db("/tmp/generic.data");
    auto pair =
      db.createTable();
    auto table = pair.first;
    ASSERT_EQ(pair.second, 0);
    ShapeShifter s{};
    table.insert(ros::Time::now(), s);
  }

  GenericDB db("/tmp/generic.data");
  auto table = db.loadTable(0);
  ros::Time t;
  ShapeShifter p;
  bool found = table.search([&](
    const ros::Time &time,
    const ShapeShifter &s){
    return true;
  },t, p);

  ASSERT_TRUE(found);
  db.erase();
}
