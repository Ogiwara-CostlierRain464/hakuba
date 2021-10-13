#include <iostream>
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include "../src/db.h"
#include "../src/tuple.h"
#include "../src/timeseries_database.h"
#include "../src/time_series_table_iter.cpp"

using namespace std;
namespace ser = ros::serialization;

struct DBTest: public ::testing::Test{};

TEST_F(DBTest, test){
  {
    DB db("/tmp/java.data", 2);
    auto pair = db.createTable();
    auto table = pair.first;
    // table id should 0
    ASSERT_EQ(pair.second, 0);

    double key = 2021921.1237;
    std::vector<uint8_t> key_vec((uint8_t*)&key, ((uint8_t *)(&key)) + sizeof(double));
    ASSERT_EQ(*reinterpret_cast<double *>(key_vec.data()), key);

    geometry_msgs::Point point;
    point.y = 1, point.z = 2;
    uint32_t serial_size = ser::serializationLength(point);
    std::vector<uint8_t> value(serial_size);

    ser::OStream stream(value.data(), serial_size);
    ser::serialize(stream, point);

    std::vector<std::vector<uint8_t>> bytes{};
    bytes.push_back(key_vec); bytes.push_back(value);
    std::vector<uint8_t> record{};
    ::tuple::encode(bytes, record);

    table.insert(record);
  }

  DB db("/tmp/java.data", 2);
  auto table = db.loadTable(0);
  Table::RefBytes out;
  bool found = table.search([&](const Table::RefBytes &record){
    std::vector<std::vector<uint8_t>> bytes{};
    std::vector<uint8_t> record_(record.begin(), record.end());
    ::tuple::decode(record_, bytes);

    geometry_msgs::Point point;
    uint32_t serial_size = ser::serializationLength(point);
    ser::IStream stream(bytes[1].data(), serial_size);
    ser::deserialize(stream, point);
    auto time = *reinterpret_cast<double*>(bytes[0].data());
    return (point.y == 1 and point.z == 2);
  }, out);

  ASSERT_TRUE(found);


  db.erase();
}

TEST_F(DBTest, test2){
  ros::NodeHandle nh_;
  {
    TimeSeriesDB db("/tmp/time.data");
    auto pair =
      db.createTable<geometry_msgs::Point>();
    auto table = pair.first;
    ASSERT_EQ(pair.second, 0);

    geometry_msgs::Point point;
    point.x = 1; point.y = 2; point.z = 3;
    table.insert(ros::Time::now(), point);
  }

  TimeSeriesDB db("/tmp/time.data");
  auto table = db.loadTable<geometry_msgs::Point>(0);
  ros::Time t;
  geometry_msgs::Point p;
  bool found = table.search([&](
    const ros::Time &time,
    const geometry_msgs::Point &point){
    return point.x == 1;
    },t, p);

  ASSERT_TRUE(found);
  db.erase();
}

TEST_F(DBTest, insert_many){
  ros::NodeHandle nh_;
  {
    TimeSeriesDB db("/tmp/many.data");
    auto pair =
      db.createTable<geometry_msgs::Point>();
    auto table = pair.first;
    ASSERT_EQ(pair.second, 0);

    for(size_t i = 0; i < 100'000; i++){
      geometry_msgs::Point point;
      point.x = (double) i; point.y = (double) (2 * i); point.z = (double)  (3 * i);
      table.insert(ros::Time::now(), point);
    }
    auto t = db.loadTable<geometry_msgs::Point>(0);
    ASSERT_TRUE(true);
    db.flush();
  }

  TimeSeriesDB db("/tmp/many.data");
  auto table = db.loadTable<geometry_msgs::Point>(0);
  ros::Time t;
  geometry_msgs::Point p;
  bool found = table.search([&](
    const ros::Time &time,
    const geometry_msgs::Point &point){
    return point.x == 5000;
  },t, p);

  ASSERT_TRUE(found);
  db.erase();
}

TEST_F(DBTest, time_iter){
  ros::NodeHandle nh_;
  {
    TimeSeriesDB db("/tmp/time_iter.data");
    auto pair =
      db.createTable<geometry_msgs::Point>();
    auto table = pair.first;
    ASSERT_EQ(pair.second, 0);

    for(size_t i = 0; i < 10'000; i++){
      geometry_msgs::Point point;
      point.x = (double) i; point.y = (double) (2 * i); point.z = (double)  (3 * i);
      table.insert(ros::Time::now(), point);
    }
    db.flush();
  }

  TimeSeriesDB db("/tmp/time_iter.data");
  auto table = db.loadTable<geometry_msgs::Point>(0);

  for(auto iter = table.begin(); iter != table.end(); ++iter){
    auto pair = *iter;
    cout << pair.first.toSec() << endl;
  }

  db.erase();
}
