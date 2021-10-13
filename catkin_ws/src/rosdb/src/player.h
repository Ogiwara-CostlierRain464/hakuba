#ifndef ROSDB_PLAYER_H
#define ROSDB_PLAYER_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <nav_msgs/Odometry.h>
#include <immintrin.h>
#include "generic_database.h"

class Player{
public:
  explicit Player(const std::string &db_path,
                  ros::NodeHandle &nh_)
  : db(db_path),
  odom_table(db.loadTable(0)),
  nh(nh_){
  }

  int run(){
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/beego/diff_drive_controller/odom", 10);

    bool is_first = true;
    ros::Time before;
    for(auto iter = odom_table.begin();
        iter != odom_table.end();
        ++iter){
      auto pair = *iter;
      if(is_first){
        before = pair.first;
        pub.publish(nav_msgs::Odometry{});
        is_first = false;
        continue;
      }

      auto tmp = ros::Time::now();
      while (pair.first - before > ros::Time::now() - tmp){
        _mm_pause();
      }
      pub.publish(nav_msgs::Odometry{});
      before = pair.first;
    }

    return 0;
  }

private:
  GenericDB db;
  GenericTable odom_table;
  ros::NodeHandle nh;
};

#endif //ROSDB_PLAYER_H
