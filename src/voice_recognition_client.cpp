//
// Created by simengangstad on 21.02.19.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "../include/core/operation/operation.h"
#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"
#include "../include/core/state.h"

std::string last_command;
bool initialized = false;

void commandCallback(const std_msgs::String::ConstPtr& string) {
    std::string new_command = std::string(string->data);

    if (last_command != new_command && initialized) {

        ROS_FATAL_STREAM("New command: " << new_command);

        fluid::OperationClient operation_client(20);

        if (new_command == "0") {
            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 1;
            take_off_pose.position.y = 1;
            take_off_pose.position.z = 1;

            operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, 
                                              take_off_pose, 
                                              [&](bool completed) {});
        }
        else if (new_command == "1") {
            geometry_msgs::Pose land_pose;
            land_pose.position.x = 1;
            land_pose.position.y = 1;
            land_pose.position.z = 0.0;

            operation_client.requestOperation(fluid::OperationIdentifier::Land, 
                                              land_pose, 
                                              [&](bool completed) {});
        }

        last_command = std::string(new_command);
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ai_voice_client");
    ros::NodeHandle node_handle;

    geometry_msgs::Pose pose;

    ros::Subscriber subscriber = node_handle.subscribe<std_msgs::String>("/ai/voice/commmand", 100, commandCallback);

    // Send an operation to initialize and arm the drone. Take off when this is done.
    fluid::OperationClient init_operation_client(20);

    init_operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        initialized = completed;
    });

    ros::Rate wait_rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Finished initialization");

	ros::spin();

    return 0;
}