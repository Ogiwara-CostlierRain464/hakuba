#ifndef SRC_DEMO_H
#define SRC_DEMO_H

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <functional>
#include <ecl/threads.hpp>
#include <ecl/utilities.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <utility>
#include <immintrin.h>

class PoseTable{
public:
    struct Pose{
        double x=0;
        double y=0;
        double theta=0;

        Pose()= default;

        Pose(double x_, double y_, double theta_):
        x(x_), y(y_), theta(theta_){}

        static void fromROSPose(const geometry_msgs::Pose &ros_pose,
                                Pose &out){
            out.x = ros_pose.position.x;
            out.y = ros_pose.position.y;
            tf::Quaternion quat(ros_pose.orientation.x,
                                ros_pose.orientation.y,
                                ros_pose.orientation.z,
                                ros_pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            out.theta = yaw;
        }
    };

    typedef std::map<ros::Time, Pose> TableType;

    explicit PoseTable()= default;

    void insert(const ros::Time &at,const Pose &pose){
        table[at] = pose;
    }

    void insert(const ros::Time &at,const geometry_msgs::Pose &pose){
        Pose my_pose;
        Pose::fromROSPose(pose, my_pose);
        table[at] = my_pose;
    }

    enum DistanceFunc{
        Euclid,
        Manhattan
    };

    void findNear(const geometry_msgs::Pose &from,
                  double distance, double theta_variance,
                  TableType &out_map,
                  DistanceFunc func = DistanceFunc::Euclid){
        Pose my_pose;
        Pose::fromROSPose(from, my_pose);
        findNear(my_pose, distance, theta_variance,
                 out_map, func);
    }

    void findNear(const Pose &from,
                  double distance, double theta_variance,
                  TableType &out_map,
                  DistanceFunc func = DistanceFunc::Euclid){
        assert(distance > 0);
        assert(theta_variance > 0);

        std::copy_if(table.begin(),
                     table.end(),
                     std::inserter(out_map, out_map.end()),
                     [from, distance, theta_variance, func](
                             decltype(table)::value_type const& kv_pair){
                         const Pose & it_pose = kv_pair.second;
                         double distance_ = 0;
                         if(func == Euclid){
                             distance_ = pow(it_pose.x - from.x, 2) + pow(it_pose.y - from.y, 2);
                         }else{
                             assert(func == Manhattan);
                             distance_ = abs(it_pose.x - from.x) + abs(it_pose.y - from.y);
                         }

                         bool check_theta = (from.theta - theta_variance) <= it_pose.theta && it_pose.theta <= (from.theta + theta_variance);

                         return (distance_ <= distance && check_theta);
        });
    }

private:
    ecl::Mutex tableMutex{};
    TableType table;
};

class ScanTable{
public:
    struct Scan{
        double angle_min;
        double angle_max;
        std::vector<double> ranges;
    };

    typedef std::map<ros::Time, Scan> TableType;

    explicit ScanTable()= default;

    void find(const ros::Time &at, Scan &out_scan){
        // do interpolation at here...
    }

private:
    ecl::Mutex tableMutex{};
    TableType table;
};

namespace {
    void user_(){
        PoseTable poseTable;
        ScanTable scanTable;

        sensor_msgs::LaserScan current_scan;
        nav_msgs::Odometry current_odom;

        PoseTable::Pose pose;
        PoseTable::Pose::fromROSPose(current_odom.pose.pose, pose);

        PoseTable::TableType map{};
        poseTable.findNear(pose, 0.5, M_PI / 12, map);


    }
}


#endif //SRC_DEMO_H
