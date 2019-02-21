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

    ROS_INFO("\nStarting Fluid FSM.");

    fluid::Core::refresh_rate = static_cast<unsigned int>(atoi(argv[1]));
    fluid::Core::message_queue_size = static_cast<unsigned int>(atoi(argv[2]));
    fluid::Core::auto_arm = std::string(argv[3]) == "true";

    fluid::OperationServer operation_server;
    operation_server.start();

    return 0;
}
