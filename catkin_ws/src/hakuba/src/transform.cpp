#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <map>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

#include "beego_controller.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

int main(int argc, char **argv){
    ros::init(argc, argv, "transform"); //ノード名の初期化
    BeegoController b;
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく
    ros::Rate loop_rate(10);

    tf::TransformBroadcaster robot_state_broadcaster;

    while(ros::ok()) {
        LaserScan scan;
        Pose pose;
        nav_msgs::OccupancyGrid grid;

        b.getCurrentPose(pose);
        TransformStamped robot_state;
        robot_state.header.stamp = ros::Time::now();
        robot_state.header.frame_id = "global";
        robot_state.child_frame_id = "robot";

        robot_state.transform.translation.x = pose.position.x;
        robot_state.transform.translation.y = pose.position.y;
        robot_state.transform.translation.z = pose.position.z;
        robot_state.transform.rotation = pose.orientation;

        robot_state_broadcaster.sendTransform(robot_state);

        ros::spinOnce();   // ここでコールバックが呼ばれる
        loop_rate.sleep(); // 10.0[Hz]で動作するように待機
    }
    return 0;
}