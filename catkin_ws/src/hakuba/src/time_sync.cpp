#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <map>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <random>
#include "beego_controller.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

struct GridMap{
    size_t height = 0;
    size_t width = 0;
    tf::TransformListener &tfListener;

    vector<int8_t> data{};


    GridMap(size_t height_, size_t width_, tf::TransformListener &l):
            height(height_), width(width_), tfListener(l){
        data.resize(height * width);
        fill(data.begin(), data.end(), 0);
    }

    void set(size_t x, size_t y, int8_t occupy){
        assert(0 <= x and x < width);
        assert(0 <= y and y < height);
        data[x + y * width] = occupy;

//        cout << "x " << x << " y " << y << endl;
    }

    void buildMessage(nav_msgs::OccupancyGrid &out_msg) const{
        out_msg.info.height = height;
        out_msg.info.width = width;

        out_msg.data = data;
    }

    laser_geometry::LaserProjection projector;

    void addScan(const Pose &pose,
                 const LaserScan &scan,
                 int occupy = 100){
        PointCloud laserPointsRobot;
        PointCloud laserPointGlobal;

        projector.projectLaser(scan, laserPointsRobot);
        laserPointsRobot.header.frame_id = "base_link";
        tfListener.transformPointCloud("odom", laserPointsRobot.header.stamp, laserPointsRobot,
                                        "base_link", laserPointGlobal);

        for(auto & point : laserPointGlobal.points){
            size_t ratio = 100;
            int x = floor(point.x * ratio) + width / 2;
            int y = floor(point.y * ratio) + height / 2;

            if(x < 0 or x >= width or y < 0 or y >= height ){
                continue;
            }
            set(x, y, occupy);
        }


    }


};



double getCurrentYawDiff(BeegoController &b, Pose &pose, Pose &first_pos){
    return b.normalize_angle(b.calcYaw(pose) - b.calcYaw(first_pos));
}

double getCurrentDistDiff(BeegoController &b,Pose &pose, Pose &first_pos){
    return hypot(pose.position.x - first_pos.position.x,
                pose.position.y - first_pos.position.y);
}

void map_check(BeegoController &b){
    auto map_pub = b.nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto map_meta_pub = b.nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);
    tf::TransformListener tf_listener;

    Pose first_pos;
    b.getCurrentPose(first_pos);
    ros::Rate loop_rate(10);

    GridMap gMap(1000, 1000, tf_listener);

    bool running = true;
    size_t state = 0;
    size_t count = 0;

    Time time_begin = Time::now();

    while(ros::ok() && running){
        LaserScan scan;
        Pose pose;
        nav_msgs::OccupancyGrid grid;

        auto elapsed = Time::now() - time_begin;
        if(elapsed.toSec() >= 30){
            break;
        }

        switch(state){
            case 0:
                b.control(10, 0.3);
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
//                gMap.addScan(pose, scan);
                if(getCurrentDistDiff(b, pose, first_pos) > 1.5 + POS_TOLERANCE){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 1;
                }
                break;
            case 1:
                assert(ros::Duration(1).sleep());
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan, -1);
                state = 2;
                break;
            case 2:
                b.control(-10, -0.3);
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                count++;
                if(count % 10 == 0){
                    gMap.addScan(pose, scan);
                }
                if(getCurrentDistDiff(b,pose, first_pos) > 1.5 + POS_TOLERANCE){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 3;
                }
                break;
            case 3:
                assert(ros::Duration(1).sleep());
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan, -1);
                gMap.buildMessage(grid);

                map_pub.publish(grid);
                map_meta_pub.publish(grid.info);
                assert(ros::Duration(1).sleep());
                running = false;
                b.stop();
                break;
        }
        ros::spinOnce();   // ここでコールバックが呼ばれる
        loop_rate.sleep(); // 10.0[Hz]で動作するように待機
    }
}

void roomba(BeegoController &b){
    auto map_pub = b.nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto map_meta_pub = b.nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);
    tf::TransformListener tf_listener;

    Pose first_pose;
    ros::Rate loop_rate(10);

    GridMap gMap(1500, 1500, tf_listener);

    // 0: running, 1: rotate
    size_t state = 0;
    size_t count = 0;

    Time time_begin = Time::now();

    while(ros::ok()){
        LaserScan scan;
        Pose pose;
        nav_msgs::OccupancyGrid grid;
        ++count;

        auto elapsed = Time::now() - time_begin;
        if(elapsed.toSec() >= 60){
            break;
        }

        b.getCurrentScan(scan);

        int i;
        switch(state){
            case 0:
                b.control(0.7, 0);
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);

                if(count >= 20){
                    count = 0;
                    gMap.addScan(pose, scan);
                }

                i = -scan.angle_min / scan.angle_increment;
                if(scan.ranges[i] < 0.9){
                    b.stop();
                    b.updateReferencePose(first_pose);
                    assert(ros::Duration(1).sleep());
                    state = 1;
                }
                break;
            case 1:
                b.control(0, 1);
                b.getCurrentPose(pose);

                if(getCurrentYawDiff(b, pose, first_pose) > (M_PI / 2)){
                    b.stop();
                    state = 0;
                }
                break;

        }
        ros::spinOnce();   // ここでコールバックが呼ばれる
        loop_rate.sleep(); // 10.0[Hz]で動作するように待機
    }

    assert(ros::Duration(1).sleep());
    OccupancyGrid  grid;
    gMap.buildMessage(grid);

    map_pub.publish(grid);
    map_meta_pub.publish(grid.info);
    assert(ros::Duration(1).sleep());
    b.stop();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_sync"); //ノード名の初期化
    BeegoController b;
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく

    roomba(b);

    return 0;
}