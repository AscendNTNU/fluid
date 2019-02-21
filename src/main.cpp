//
// Created by simengangstad on 27.09.18.
//

#include "../include/actionlib/operation_server.h"
#include <ros/ros.h>
#include <iostream>
#include <memory>

#include "../include/core/core.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");

    ROS_INFO("\n Starting Fluid FSM...");

    fluid::Core::refresh_rate = atoi(argv[1]);
    fluid::Core::message_queue_size = atoi(argv[2]);
    fluid::Core::auto_arm = argv[3] == "true";

    fluid::OperationServer operation_server;
    operation_server.start();

    ROS_INFO("Operation server up and running.\n");

    return 0;
}
