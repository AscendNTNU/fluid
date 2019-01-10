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
    ros::NodeHandle nh("~");

    geometry_msgs::Pose pose;
    bool initialized = false;

    float height = 1.5;

    ROS_INFO("Starting init operation.");
    // Send an operation to initialize and arm the drone. Take off when this is done.
    fluid::OperationClient init_operation_client(20);
    
    init_operation_client.requestOperation(fluid::operation_identifiers::INIT, pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Init operation completed");

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 0;
            take_off_pose.position.y = 0;
            take_off_pose.position.z = height;


            ROS_INFO("Starting take off operation.");
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

    ROS_INFO("Completed initialization and take off");    
    ROS_INFO("Starting with interval calls");

    fluid::OperationClient move_operation_client(0.5);
    int flip = 1;

    ros::Rate rate(3);

    while (ros::ok()) {
            // ROS_INFO_STREAM("Moving to: " << pose.position);    

            pose.position.y += 0.1;
            pose.position.z = height;
            flip = -flip;
            move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [](bool completed) {});

            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}

