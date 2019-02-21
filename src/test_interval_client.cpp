#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <chrono>
#include <ostream>

#include "../include/core/operation/operation.h"
#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_defines.h"
#include "../include/core/state.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh;


    // Set boundries
    nh.setParam("boundryX", 2);
    nh.setParam("boundryY", 2);
    nh.setParam("boundryZ", 1);

    geometry_msgs::Pose pose;
    bool initialized = false;

    float height = 1.5;

    // Send an operation to initialize and arm the drone. Take off when this is done.
    fluid::OperationClient init_operation_client(20);
    
    init_operation_client.requestOperation(fluid::operation_identifiers::INIT, pose, [&](bool completed) {
        if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 0;
            take_off_pose.position.y = 0;
            take_off_pose.position.z = height;

            fluid::OperationClient take_off_operation_client(20);

            take_off_operation_client.requestOperation(fluid::operation_identifiers::TAKE_OFF, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });

    ros::Rate wait_rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    fluid::OperationClient move_operation_client(5);

    ros::Rate rate(20);

    while (ros::ok()) {

        pose.position.x += 0.01;
        pose.position.z = height;

        move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [](bool completed) {});

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

