#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <map>
#include <vector>

#include "beego_controller.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;

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

        cout << "x " << x << " y " << y << endl;
    }

    void buildMessage(nav_msgs::OccupancyGrid &out_msg) const{
        out_msg.info.height = height;
        out_msg.info.width = width;

        out_msg.data = data;
    }

    static void fromScan(size_t height, size_t width,
                         const Pose &pose,
                         const LaserScan &scan, GridMap &out_map){
        out_map = GridMap(height, width);

        auto current_angle = scan.angle_min;
        for(auto scan_distance : scan.ranges){
            size_t ratio = 100;
            assert(scan_distance > 0);

            float x = scan_distance * cos(current_angle);
            float y = scan_distance * sin(current_angle);

            // convert to global coordinate
            x += pose.position.x;
            y += pose.position.y;

            tf::Quaternion quat(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            auto theta = yaw;
            auto g_x = cos(theta) * x - sin(theta) * y;
            auto g_y = sin(theta) * x + cos(theta) * y;

            // convert to Grid map coordinate
            int x2 = floor(g_x * ratio) + (width / 2);
            int y2 = floor(g_y * ratio) + (height / 2);

            if(x2 < 0 or x2 >= width or y2 < 0 or y2 >= height ){
                continue;
            }

            out_map.set(x2, y2, 100);
            assert(scan.angle_increment > 0);
            current_angle += scan.angle_increment;
        }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_sync"); //ノード名の初期化
    BeegoController b;
    ros::Duration(1.0).sleep(); // 1.0秒待機
    ros::spinOnce(); // はじめにコールバック関数を呼んでおく

    auto map_pub = b.nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    auto map_meta_pub = b.nh_.advertise<nav_msgs::MapMetaData>("/map_metadata", 1, true);

    size_t count = 0;

    while(ros::ok() and count < 4){
        sensor_msgs::LaserScan scan;
        b.getCurrentScan(scan);
        geometry_msgs::Pose pose;
        b.getCurrentPose(pose);

        constexpr size_t HEIGHT = 1000;
        constexpr size_t WIDTH = 1000;

        GridMap gMap;
        GridMap::fromScan(HEIGHT, WIDTH, pose, scan, gMap);

        nav_msgs::OccupancyGrid grid;
        gMap.buildMessage(grid);

        map_pub.publish(grid);
        map_meta_pub.publish(grid.info);

        ros::Duration(1.0).sleep();

        ++count;
    }
    return 0;
}