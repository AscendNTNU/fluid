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

    auto reference = std::chrono::high_resolution_clock::now();
    const int radius = 2.5;

    double time = 0.0;

    while (ros::ok()) {

        pose.position.y = radius * sin(time / 2000.0);
        pose.position.x = radius * cos(time / 2000.0);
        pose.position.z = height;

        move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [](bool completed) {});

        ros::spinOnce();
        rate.sleep();

        time += static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - reference).count());
    }

    return 0;
}

