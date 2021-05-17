#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include "beego_controller.h"

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

bool moveToGoad(const geometry_msgs::Point &pos, const geometry_msgs::Quaternion &quat){
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
            ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position = pos;
    goal.target_pose.pose.orientation = quat;

    ROS_INFO("Sending goal location ...");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the destination");
        return true;
    }else{
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "play");
    BeegoController b{};
    ros::Duration(1.0).sleep();
    ros::NodeHandle n;
    ros::spinOnce();
    // STEP1: go to fork point
    Point p; Quaternion q;
    p.x = 0.85; p.y = -0;
    q.z = -0.7; q.w = 0.7;
    moveToGoad(p, q);

    // STEP2: check either left or right is open.
    bool left;

    LaserScan scan;
    b.getCurrentScan(scan);
    size_t middle = -scan.angle_min / scan.angle_increment;

    size_t deg = (3.14 / 4) / abs(scan.angle_increment);
    cout << deg << endl;
    cout << scan.ranges[middle - deg] << endl; // left
    cout << scan.ranges[middle + deg] << endl; // right
    if(scan.ranges[middle - deg] > 2){
        left = true;
    }else{
        left = false;
    }
    // STEP3: switch statement.

    if(left){
        // before OJYAMA robot.
        p.x = 1.5; p.y = -0.9;
        q.z = -0.7; q.w = 0.7;
        moveToGoad(p, q);

        // go straight
        p.x = 1.4; p.y = -2.5;
        q.z = -0.7; q.w = 0.7;
        moveToGoad(p, q);
    }else{
        // before OJYAMA robot.
        p.x = 0.5; p.y = -0.9;
        q.z = -0.7; q.w = 0.7;
        moveToGoad(p, q);

        // go straight
        p.x = 0.4; p.y = -2.5;
        q.z = -0.7; q.w = 0.7;
        moveToGoad(p, q);
    }

    // STEP 4: right or warehouse C
    p.x = -0.9; p.y = -2.9;
    q.z = -0.99; q.w = 0;
    moveToGoad(p, q);


    // STEP 5: in front of warehouse C
    p.x = -0.85; p.y = -2.4;
    q.z = 0.7; q.w = 0.7;
    moveToGoad(p, q);

    // STEP 6: go straight

    geometry_msgs::Pose ref_pose;
    geometry_msgs::Pose pose;

    b.updateReferencePose(ref_pose);
    for(;;){
        b.getCurrentPose(pose);
        double dist_diff = hypot(pose.position.x - ref_pose.position.x,
                                 pose.position.y - ref_pose.position.y);

        b.straight();
        if(dist_diff > 0.7){
            break;
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    return 0;
}