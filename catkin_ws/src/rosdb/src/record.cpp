#include <ros/ros.h>
#include "recorder.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "record",  ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if(!nh.ok())
    exit(-1);
  ros::Time::waitForValid();

  Recorder recorder("/tmp/ros_db.data", nh);
  return recorder.run();
}