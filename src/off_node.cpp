#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "../include/core/operation/operation.h"
#include "actionlib/operation_client.h"
#include "../include/core/state.h"

/*
mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}*/

void move(int x, int y, int z) {
    fluid::OperationClient operation_client(fluid::OperationIdentifier::move, 5);

    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    operation_client.requestOperationToTargetPoint(pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Operation completed (callback)");
        }
        else {
            ROS_INFO("Operation failed (callback)");
        }
    });
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "client");

    ros::NodeHandle nh("~");

    fluid::OperationClient operation_client(fluid::OperationIdentifier::init, 5);

    geometry_msgs::Pose pose;

    operation_client.requestOperationToTargetPoint(pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Operation completed (callback)");
        }
        else {
            ROS_INFO("Operation failed (callback)");
        }
    });



    ros::Duration(10).sleep();

    fluid::OperationClient operation_take_off_client(fluid::OperationIdentifier::take_off, 5);

    geometry_msgs::Pose take_off_pose;
    take_off_pose.position.x = 0;
    take_off_pose.position.y = 0;
    take_off_pose.position.z = 2;

    operation_take_off_client.requestOperationToTargetPoint(take_off_pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Operation completed (callback)");
        }
        else {
            ROS_INFO("Operation failed (callback)");
        }
    });

    ros::Duration(7).sleep();

    move(5, 0, 2);
    ros::Duration(3).sleep();

    move(5, 5, 2);
    ros::Duration(3).sleep();

    move(0, 5, 2);
    ros::Duration(3).sleep();

    move(0, 0, 2);
    ros::Duration(3).sleep();

    fluid::OperationClient operation_land_client(fluid::OperationIdentifier::land, 5);

    geometry_msgs::Pose land_pose;
    land_pose.position.x = 0;
    land_pose.position.y = 0;
    land_pose.position.z = 0;

    operation_land_client.requestOperationToTargetPoint(land_pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Operation completed (callback)");
        }
        else {
            ROS_INFO("Operation failed (callback)");
        }
    });


    return 0;
}

