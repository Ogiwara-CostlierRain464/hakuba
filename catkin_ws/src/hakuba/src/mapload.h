#ifndef HAKUBA_MAPLOAD_H
#define HAKUBA_MAPLOAD_H

#include <cstdio>
#include <vector>
#include <set>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

struct Field
{
    int seq;
    geometry_msgs::Point32 robot_pos;
    std::map<size_t,
    std::vector<geometry_msgs::Point32>> landmarks;

};

sensor_msgs::PointCloud landmarks_msg_;
geometry_msgs::PoseArray captured_poses_msg_;
std::vector<Field> map_;

bool getMap(const std::string& filename)
{
    std::ifstream ifs;
    ifs.open(filename);
    if (!ifs) {
        ROS_INFO("[particle filter] cannot open map file");
        return false;
    }

    Field field;
    int now_type = -1;
    std::string str;
    geometry_msgs::Point32 robot_pose;
    std::vector<std::string> line;
    while (getline(ifs, str)) {
        line.clear();
        std::stringstream ss(str);
        while(getline(ss, str, ',')) {
            line.push_back(str);
        }

        if (line[0] == "header") { // header, seq, x, y, theta
            field.seq = stoi(line[1]);
            robot_pose.x = stof(line[2]);
            robot_pose.y = stof(line[3]);
            robot_pose.z = stof(line[4]);

            // create captured poses msg
            geometry_msgs::Pose pose;
            geometry_msgs::Quaternion quat;
            pose.position.x = robot_pose.x;
            pose.position.y = robot_pose.y;
            pose.position.z = 0.0;
            tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0.0, 0.0, robot_pose.z), quat);
            pose.orientation = quat;
            captured_poses_msg_.poses.push_back(pose);

            // 追加
            field.robot_pos = robot_pose;
        }
        else if (line[0] == "pos") { // pos, type, pos_x, pos_y
            int type = stoi(line[1]);
            geometry_msgs::Point32 point;
            point.x = stof(line[2]);
            point.y = stof(line[3]);
            point.z = 0.0;
            field.landmarks[type].push_back(point);

            // create landmarks msg
            geometry_msgs::Point32 global_point;
            double length = sqrt( point.x * point.x + point.y * point.y );
            double angle = atan2( point.y, point.x ) + robot_pose.z;
            global_point.x = robot_pose.x + length * cos(angle);
            global_point.y = robot_pose.y + length * sin(angle);
            global_point.z = 0.0;


        }
        else if (line[0] == "end") {
            map_.push_back(field);
            field.landmarks.clear();
        }
    }
    ROS_INFO("Get Map");
    // 地図上の現在位置を取得
    // initializeMapIndex();
    return true;
}



#endif //HAKUBA_MAPLOAD_H
