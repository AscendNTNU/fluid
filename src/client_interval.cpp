#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "../include/core/operation/operation.h"
#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_interval");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;
    bool initialized = false;
    float height = 1.0;

    fluid::OperationClient operation_client("drone_1", 60);
    
    operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 0;
            take_off_pose.position.y = 0;
            take_off_pose.position.z = height;

            operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });

    ros::Rate wait_rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ros::Rate rate(20);

    pose.position.z = height;

    while (ros::ok()) {

        pose.position.x += 0.01;
        pose.position.y += 0.01;
        pose.position.z += 0.01;

        operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [](bool completed) {});

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

