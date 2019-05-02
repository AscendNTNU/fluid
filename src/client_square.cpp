#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <chrono>
#include <ostream>
#include <string>

#include <cstdlib> 
#include <ctime>     
#include <memory>

#include "../include/core/operation/operation.h"
#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"
#include "../include/core/state.h"

std::string name_space;
float xLength, yLength;
std::shared_ptr<fluid::OperationClient> operation_client_ptr;

bool initialPoseSet = false;
geometry_msgs::Pose initialPose;
geometry_msgs::Pose lastPose;
geometry_msgs::Pose pose;

constexpr float height = 2.5;

void runOperation(bool completed ignored for this purpose) {

    // Run two operations:
    // 
    // 1. Rotate towards the next position
    // 2. Move to the next position

    float x = initialPose.position.x + (-xLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(xLength * 2))));
    float y = initialPose.position.y + (-yLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(yLength * 2))));

    ROS_INFO_STREAM("Current pose: " << "(" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")");
    ROS_INFO_STREAM("Next position: " << "(" << x << ", " << y << ", " << pose.position.z << ")");

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = height;

    operation_client_ptr->requestOperation(fluid::OperationIdentifier::MoveOriented, pose, runOperation);
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {

    if (!initialPoseSet && fabs(pose->pose.position.x) > 0.5) {

        initialPoseSet = true;
        initialPose = pose->pose;
        lastPose = initialPose;
    }
}

int main(int argc, char** argv) {

    name_space = std::string(argv[1]);
    xLength = static_cast<float>(atof(argv[2]));
    yLength = static_cast<float>(atof(argv[3]));

    ros::init(argc, argv, name_space + "_random_movement_client");
    ros::NodeHandle node_handle;
    operation_client_ptr = std::make_shared<fluid::OperationClient>(name_space, 60);
    ros::Rate rate(20);

    // Retrieve initial pose 
    ros::Subscriber subscriber = node_handle.subscribe(name_space + "/mavros/local_position/pose", 100, poseCallback);

    while (ros::ok() && !initialPoseSet) {
        ros::spinOnce();
        rate.sleep();
    }


    // Initialization and take off
    bool initialized = false;
    
    
    pose.position.x = initialPose.position.x;
    pose.position.y = initialPose.position.y;


    pose.position.x = 0.0;
    pose.position.y = 0.0;

    operation_client_ptr->requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            pose.position.z = height;

            operation_client_ptr->requestOperation(fluid::OperationIdentifier::TakeOff, pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize the random movement
    srand(static_cast<unsigned int>(time(nullptr)));
    runOperation(true);
    ros::spin();

    return 0;
}