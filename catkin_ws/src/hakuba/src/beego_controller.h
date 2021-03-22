#ifndef BEEGO_CONTROLLER_H
#define BEEGO_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

#define POS_TOLERANCE 0.001        // 状態遷移時に許容する位置誤差 [m]
#define RAD_TOLERANCE M_PI / 180.0 // 状態遷移時に許容する角度誤差 [rad]
#define LINEAR_VEL 0.15            // ロボットの並進速度 [m/s]
#define ANGULAR_VEL 0.5            // ロボットの角速度 [rad/s]

class BeegoController
{
public:
    BeegoController();
    // 関数
    void odomCallback(const nav_msgs::Odometry::ConstPtr &);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgs);
    void straight(void) const;
    void spinRight(void);
    void spinLeft(void);
    void stop(void);
    void control(double x, double yaw);
    double calcYaw(geometry_msgs::Pose);
    void updateReferencePose(geometry_msgs::Pose& pose);
    void getCurrentPose(geometry_msgs::Pose& pose);
    void getCurrentScan(sensor_msgs::LaserScan& scan);
    double normalize_angle(double angle);

    // ROS関係の変数
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

    // コールバックで更新される変数
    geometry_msgs::Pose pose_; // 現在の位置・姿勢
    sensor_msgs::LaserScan scan_; // 現在のセンサ情報

    std::string LiDAR_topic;
    std::string odom_topic;

};

#endif