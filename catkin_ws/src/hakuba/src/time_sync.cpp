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

struct GridMap{
    size_t height = 0;
    size_t width = 0;

    vector<int8_t> data{};

    GridMap() = default;

    GridMap(size_t height_, size_t width_):
            height(height_), width(width_){
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
                 const LaserScan &scan){
        auto current_angle = scan.angle_min;
        for(auto scan_distance : scan.ranges){

            tf::TransformListener tf_listener;
            PointCloud laserPointsRobot;
            PointCloud laserPointGlobal;

            projector.projectLaser(scan, laserPointsRobot);
            tf_listener.transformPointCloud("robot", laserPointsRobot.header.stamp, laserPointsRobot,
                                            "global", laserPointGlobal);

            for(auto & point : laserPointGlobal.points){

            }


            size_t ratio = 70;
            // assert(scan_distance > 0);

            float x = scan_distance * cos(current_angle);
            float y = scan_distance * sin(current_angle);


//            x -= pose.position.x;
//            y -= pose.position.y;

            tf::Quaternion quat(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            double result = yaw;
            while (result > M_PI)
                result -= 2.0 * M_PI;
            while (result < -M_PI)
                result += 2.0 * M_PI;

            double g_x = cos(result) * x - sin(result) * y;
            double g_y = sin(result) * x + cos(result) * y;
//             double g_x = x, g_y = y;

                        // convert to global coordinate
            cout << pose.position << endl;

//            g_x -= pose.position.x;
//            g_y -= pose.position.y;

            // i have to solve inversed problem?

            // convert to Grid map coordinate
            int x2 = floor(g_x * ratio) + (width / 2);
            int y2 = floor(g_y * ratio) + (height / 2);
//            int x2 = floor(g_x - (float)width / 2) * ratio;
//            int y2 = floor(g_y - (float)height / 2) * ratio;


            if(x2 < 0 or x2 >= width or y2 < 0 or y2 >= height ){
                continue;
            }

            //　だるいし、先に完成させる？DB
            // lookup data by most nearest time
            // just using map...
            // another step?
            // to support view!  recent,
            // how to implement interface?
            // think about Mauve DB
            // allow programmer to look inside b-tree? : NO.
            // just offer several methods to user

            set(x2, y2, 100);
            assert(scan.angle_increment > 0);
            current_angle += scan.angle_increment;
        }
    }

    static void fromScan(size_t height, size_t width,
                         const Pose &pose,
                         const LaserScan &scan, GridMap &out_map){
        out_map = GridMap(height, width);
        out_map.addScan(pose, scan);
    }

};

struct DB{
    map<ros::Time, sensor_msgs::LaserScan> lidarScan;

    void greatest_less(const Time &time, LaserScan &out_scan){
        auto it = lidarScan.lower_bound(time);
        if(it != lidarScan.begin()){
            out_scan = it->second;
        }
        out_scan = lidarScan.end()->second;
    }
};

double getCurrentYawDiff(BeegoController &b, Pose &pose, Pose &first_pos){
    return b.normalize_angle(b.calcYaw(pose) - b.calcYaw(first_pos));
}

double getCurrentDistDiff(BeegoController &b,Pose &pose, Pose &first_pos){
    return hypot(pose.position.x - first_pos.position.x,
                pose.position.y - first_pos.position.y);
}

void walk_stop_scan(BeegoController &b){
    auto map_pub = b.nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto map_meta_pub = b.nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);

    Pose first_pos;
    b.getCurrentPose(first_pos);
    ros::Rate loop_rate(10);

    GridMap gMap(1000, 1000);

    bool running = true;
    size_t state = 0;
    while(ros::ok() && running){
        LaserScan scan;
        Pose pose;

        switch(state){
            case 0:
//                b.control(0, -0.3);
                b.control(0.2, 0);
                b.getCurrentPose(pose);
//                if(getCurrentYawDiff(b, pose, first_pos) < (-M_PI / 3)){
                if(getCurrentDistDiff(b, pose, first_pos) > 1){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 1;
                }
                break;
            case 1:
                ros::Duration(3).sleep();
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan);
                state = 2;
                break;
            case 2:
//                b.control(0, 0.3);
                b.control(-0.2, 0);
                b.getCurrentPose(pose);
//                if(getCurrentYawDiff(b, pose, first_pos) > (M_PI / 3)){
                if(getCurrentDistDiff(b,pose, first_pos) > 1){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 3;
                }
                break;
            case 3:
                ros::Duration(3).sleep();
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan);
                nav_msgs::OccupancyGrid grid;
                gMap.buildMessage(grid);

                map_pub.publish(grid);
                map_meta_pub.publish(grid.info);
                running = false;
                b.stop();
                break;
        }
        ros::spinOnce();   // ここでコールバックが呼ばれる
        loop_rate.sleep(); // 10.0[Hz]で動作するように待機
    }
}

void map_check(BeegoController &b){
    auto map_pub = b.nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto map_meta_pub = b.nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);

    Pose first_pos;
    b.getCurrentPose(first_pos);
    ros::Rate loop_rate(10);

    GridMap gMap(1000, 1000);

    bool running = true;
    size_t state = 0;

    tf::TransformBroadcaster robot_state_broadcaster;

    while(ros::ok() && running){
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


        switch(state){
            case 0:
                b.control(0, -0.3);
//                b.control(0.2, 0);
                b.getCurrentPose(pose);
                if(getCurrentYawDiff(b, pose, first_pos) < (-M_PI / 3) + RAD_TOLERANCE){
//                if(getCurrentDistDiff(b, pose, first_pos) > 1 + POS_TOLERANCE){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 1;
                }
                break;
            case 1:
                assert(ros::Duration(3).sleep());
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan);
                state = 2;
                break;
            case 2:
                   b.control(0, 0.3);
//                b.control(-0.2, 0);
                b.getCurrentPose(pose);
                if(getCurrentYawDiff(b, pose, first_pos) > (M_PI / 3) + RAD_TOLERANCE){
//                if(getCurrentDistDiff(b,pose, first_pos) > 1 + POS_TOLERANCE){
                    b.stop();
                    b.updateReferencePose(first_pos);
                    state = 3;
                }
                break;
            case 3:
                assert(ros::Duration(3).sleep());
                b.getCurrentPose(pose);
                b.getCurrentScan(scan);
                gMap.addScan(pose, scan);
                gMap.buildMessage(grid);

                map_pub.publish(grid);
                map_meta_pub.publish(grid.info);
                assert(ros::Duration(0.5).sleep());
                running = false;
                b.stop();
                break;
        }
        ros::spinOnce();   // ここでコールバックが呼ばれる
        loop_rate.sleep(); // 10.0[Hz]で動作するように待機
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_sync"); //ノード名の初期化
    BeegoController b;
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく

    map_check(b);

    return 0;
}