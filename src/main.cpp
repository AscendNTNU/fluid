//
// Created by simengangstad on 27.09.18.
//

#include "../include/actionlib/operation_server.h"
#include <ros/ros.h>
<<<<<<< HEAD
#include <ostream>
=======
#include <iostream>
#include <memory>
>>>>>>> develop

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");

<<<<<<< HEAD
    std::cout << "Hello World" << std::endl;

    fluid::OperationServer operation_server;
    operation_server.execute();

=======
    fluid::OperationServer operation_server(atoi(argv[1]));
    operation_server.start();
    
>>>>>>> develop
    return 0;
}
