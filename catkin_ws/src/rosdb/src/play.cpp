#include <iostream>
#include <ros/ros.h>
#include "generic_database.h"
#include "player.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "record",  ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if(!nh.ok())
    exit(-1);
  ros::Time::waitForValid();

  Player player("/tmp/ros_db.data", nh);
  return player.run();
}