#ifndef ROSDB_RECORDER_H
#define ROSDB_RECORDER_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "generic_database.h"

// 0: Twist
// 1: PoseWithCovariance
class Recorder{
public:
  explicit Recorder(const std::string &save_path,
                    ros::NodeHandle &nh_)
  : db(save_path),
  odom_table(db.createTable().first),
  nh(nh_){}

  void callback_odom(const topic_tools::ShapeShifter &msg){
    odom_table.insert(ros::Time::now(), msg );
  }

  int run(){
    auto first = ros::Time::now();

    ros::Subscriber subscriber =
      nh.subscribe("/beego/diff_drive_controller/odom", 10, &Recorder::callback_odom, this);

    while (nh.ok()){
      ros::spinOnce();

      auto now = ros::Time::now();
      if((now - first).toSec() > 5){
        break;
      }
    }
    db.flush();

    return 0;
  }

private:
  GenericDB db;
  GenericTable odom_table;
  ros::NodeHandle &nh;
};

#endif //ROSDB_RECORDER_H
