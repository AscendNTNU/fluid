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

    ROS_INFO("After init operation");

    ros::Rate wait_rate(20);


    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Completed initialization and take off");    
    ROS_INFO("Starting with interval calls");

    fluid::OperationClient move_operation_client(1);
    ros::Rate rate(2);
    int flip = 1;

    while (ros::ok()) {
            ROS_INFO_STREAM("Moving to: " << pose.position);    

            pose.position.y += 0.2;
            pose.position.z = height;
            flip = -flip;
            ROS_INFO("Before move operaiton call");
            move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [](bool completed) {});
            ROS_INFO("After move operation call");


            // TODO: When we crank down the interval the hold state doesn't get its position target set properly (maybe because of the multiple threads simoultanously). Need to figure out a smooth way to have a short interval. 

            // TODO: Something funky is happening with the callback when waiting for the result in a 
            // separate thread.

            rate.sleep();
    }

    return 0;
}

