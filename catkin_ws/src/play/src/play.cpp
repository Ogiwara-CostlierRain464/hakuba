#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace geometry_msgs;

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
    }
    else{
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "play");
    ros::NodeHandle n;
    ros::spinOnce();
    // go
    // x: -2.6 y: -4.21
    // z: -1  w: 0.17
    Point p; Quaternion q;
    p.x = 0.85; p.y = -0;
    q.z = -0.67; q.w = 0.73;
    moveToGoad(p, q);

    //check lidar scan at here


    // x: 0.5, y: -0.9  | z: -0.7, w: 0.7
    p.x = 0.5; p.y = -0.9;
    q.z = -0.7; q.w = 0.7;
    moveToGoad(p, q);


    // x: 0.5, y: -2.29 | z: -0.7, w: 0.7
    p.x = 0.4; p.y = -2.7;
    q.z = -0.7; q.w = 0.7;
    moveToGoad(p, q);

    // x: -0.9, y* -2.9 | z: 0.99, w: 0
    p.x = -0.9; p.y = -2.9;
    q.z = -0.99; q.w = 0;
    moveToGoad(p, q);


    // x: -0.9, y* -2.4 | z: 0.7, w: 0.7
    p.x = -0.85; p.y = -2.4;
    q.z = 0.7; q.w = 0.7;
    moveToGoad(p, q);

    return 0;
}