//
// Created by simengangstad on 18.10.18.
//

#include "../../include/actionlib/action_server.h"
#include <actionlib/server/simple_action_server.h>
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/LandAction.h>
#include <fluid_fsm/TakeOffAction.h>
#include <ros/ros.h>

fluid::ActionServer::ActionServer(fluid::OperationIdentifier operation_identifier) :
                                       operation_identifier_(operation_identifier) {
    ros::NodeHandle node_handle;
    Server server(node_handle, operation_identifier, boost::bind(&execute, _1, &server), false);
    server.start();
}

template <typename  GoalConstPtr, typename Action>
void fluid::ActionServer::execute(const GoalConstPtr &goal, actionlib::SimpleActionServer <Action> *action_server_p) {
    // Send final pose in set succeeded
    action_server_p->setSucceeded();
}