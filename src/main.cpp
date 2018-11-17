//
// Created by simengangstad on 27.09.18.
//

#include "../include/actionlib/operation_server.h"
#include <ros/ros.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");

    fluid::OperationServer operation_server;
    operation_server.execute();

    return 0;
}
