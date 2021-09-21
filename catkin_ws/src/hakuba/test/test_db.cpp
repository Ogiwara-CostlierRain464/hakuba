#include <iostream>
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <geometry_msgs/Pose.h>
#include "../src/db/db.h"
#include "../src/db/tuple.h"

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
