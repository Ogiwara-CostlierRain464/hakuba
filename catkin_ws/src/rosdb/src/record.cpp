#include <ros/ros.h>
#include "recorder.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "record",  ros::init_options::AnonymousName);
  Recorder recorder("/tmp/ababa");
  return recorder.run();
}