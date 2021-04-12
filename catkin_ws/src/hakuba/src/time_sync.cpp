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
#include "db.h"
#include "demo.h"
#include <ecl/threads.hpp>
#include <ctime>

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

void demo(BeegoController &b){

    PoseTable poseTable;
    ros::Rate loop_rate(10);
    size_t count = 0;
    while(ros::ok()){
        Pose current_pose; b.getCurrentPose(current_pose);
        auto now = Time::now(); poseTable.insert(now, current_pose);
        count++;

        b.control(1, 0.5);

        if(count == 200){
            // find last time that was close to current position.
            PoseTable::TableType map;
            poseTable.findNear(current_pose, // looking for near this pose
                               0.1,  // within this distance
                               M_PI / 30, // within is theta variance
                               map); // search result

            for(auto & kv : map){
                // printing which time was near to current pose
                cout << kv.first.sec << endl;
            }
            b.stop();
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_sync"); //ノード名の初期化
    BeegoController b;
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく
    ros::Time::waitForValid();

    demo(b);

    return 0;
}