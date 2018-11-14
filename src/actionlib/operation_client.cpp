#include <utility>

//
// Created by simengangstad on 25.10.18.
//

#include "actionlib/operation_client.h"

#include <fluid_fsm/OperationAction.h>
#include <fluid_fsm/OperationGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <actionlib/operation_client.h>

void fluid::OperationClient::requestOperation(fluid::OperationIdentifier operation_identifier,
                                              geometry_msgs::Pose target_pose,
                                              std::function<void(bool)> completion_handler) {

    actionlib::SimpleActionClient<fluid_fsm::OperationAction> action_client("fluid_fsm_operation", true);
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
