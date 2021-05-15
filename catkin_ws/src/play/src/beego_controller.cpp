#include "beego_controller.h"

BeegoController::BeegoController() : nh_("~")
{
    if(!nh_.getParam("lidar_topic", LiDAR_topic)){
         LiDAR_topic = "/beego/scan";
//        LiDAR_topic = "/scan";
    }
    if(!nh_.getParam("odom_topic", odom_topic)){
         odom_topic = "/beego/diff_drive_controller/odom";
//        odom_topic = "/odom";
    }
    if(!nh_.getParam("cmd_vel_topic", cmd_vel_topic)){
         cmd_vel_topic = "/beego/diff_drive_controller/cmd_vel";
//        cmd_vel_topic = "/cmd_vel";
    }

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    odom_sub_ = nh_.subscribe(odom_topic, 10, &BeegoController::odomCallback, this);
    scan_sub_ = nh_.subscribe(LiDAR_topic, 10, &BeegoController::scanCallback, this);
}

void BeegoController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pose_ = msg->pose.pose;
}

void BeegoController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgs){
    scan_ = *msgs;
}

void BeegoController::straight() const
{
    geometry_msgs::Twist msg;
    msg.linear.x = LINEAR_VEL;
    msg.angular.z = 0.0;
    vel_pub_.publish(msg);
}

void BeegoController::spinRight()
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = -ANGULAR_VEL;
    vel_pub_.publish(msg);
}

void BeegoController::spinLeft(void)
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = ANGULAR_VEL;
    vel_pub_.publish(msg);
}

void BeegoController::stop(void)
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    vel_pub_.publish(msg);
}

void BeegoController::control(double x, double yaw)
{
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.angular.z = yaw;
    vel_pub_.publish(msg);
}

double BeegoController::calcYaw(geometry_msgs::Pose pose)
{ // geometry_msgs::PoseからYaw角を計算する関数
    tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

void BeegoController::updateReferencePose(geometry_msgs::Pose& ref_pose)
{ // 基準となる位置や姿勢を更新する関数
    ref_pose = pose_;
}

void BeegoController::getCurrentPose(geometry_msgs::Pose& pose)
{ // 現在の姿勢を取得する関数
    pose = pose_;
}

void BeegoController::getCurrentScan(sensor_msgs::LaserScan& scan)
{ // 現在のセンサ情報を取得する関数
    scan = scan_;
}

double BeegoController::normalize_angle(double angle)
{ // 角度を-π〜+πに正規化する関数
    double result = angle;
    while (result > M_PI)
        result -= 2.0 * M_PI;
    while (result < -M_PI)
        result += 2.0 * M_PI;
    return result;
}