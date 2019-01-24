//
// Created by simengangstad on 27.09.18.
//

#include "../include/actionlib/operation_server.h"
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include "../include/core/operation/state_graph.h"
#include <vector>
#include "../include/core/state.h"
#include "../include/states/state_defines.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");
    ros::NodeHandle nh;

    fluid::StateGraph graph;

    graph.configure(20);

    std::vector<std::shared_ptr<fluid::State>> plan = graph.getPathToEndState(fluid::state_identifiers::INIT, 
    																   		  fluid::state_identifiers::IDLE);

    for (auto state : plan) {
    	std::cout << state->identifier << std::endl;
    }
    
    return 0;
}
