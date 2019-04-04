//
// Created by simengangstad on 25.10.18.
//

#include "../../include/actionlib/operation_client.h"

#include <actionlib/operation_client.h>
#include <fluid_fsm/OperationGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <utility>
#include <thread>


void fluid::OperationClient::waitForResult(
    std::string operation_identifier,
    geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    Client action_client(name_space + "/fluid_fsm_operation", false);

    action_client.waitForServer();

    fluid_fsm::OperationGoal goal;
    goal.target_pose = target_pose;
    std_msgs::String type;
    type.data = std::move(operation_identifier);
    goal.type = type;
    action_client.sendGoal(goal);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(timeout_value_));

    if (completion_handler) {
        completion_handler(finished_before_timeout && action_client.getState().isDone());
    }
}

void fluid::OperationClient::requestOperation(
    std::string operation_identifier,
	geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    std::thread thread(&OperationClient::waitForResult, this, operation_identifier, target_pose, completion_handler);
    thread.detach();
}
