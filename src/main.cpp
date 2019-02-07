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

    fluid::Core::refresh_rate = atoi(argv[1]);
    fluid::Core::message_queue_size = atoi(argv[2]);

    fluid::OperationServer operation_server;
    operation_server.start();
    
    return 0;
}
