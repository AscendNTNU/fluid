//
// Created by simengangstad on 25.10.18.
//

#include "../../include/actionlib/operation_client.h"

#include <actionlib/operation_client.h>
#include <boost/thread.hpp>
#include <fluid_fsm/OperationGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <utility>
#include <thread>



void fluid::OperationClient::waitForResult(
    std::string operation_identifier,
    geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {
}

void fluid::OperationClient::requestOperation(
    fluid::OperationIdentifier operation_identifier,
	geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    //boost::thread thread(boost::bind(&OperationClient::waitForResult, this, operation_identifier, target_pose, completion_handler));

    fluid_fsm::OperationGoal goal;
    goal.target_pose = target_pose;
    std_msgs::String type;
    type.data = std::move(operation_identifier);
    goal.type = type;
    action_client_.sendGoal(goal);

    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(timeout_value_));

    if (completion_handler) {
        completion_handler(finished_before_timeout && action_client_.getState().isDone());
    }
}
