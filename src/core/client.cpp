//
// Created by simengangstad on 25.10.18.
//

#include "client.h"

#include <fluid/OperationGoal.h>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <utility>
#include <thread>


void fluid::Client::waitForResult(
    std::string operation_identifier,
    geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    ActionlibClient actionlib_client(name_space + "/fluid_fsm_operation", false);

    actionlib_client.waitForServer();

    fluid::OperationGoal goal;
    goal.target_pose = target_pose;
    std_msgs::String type;
    type.data = std::move(operation_identifier);
    goal.type = type;
    actionlib_client.sendGoal(goal);

    bool finished_before_timeout = actionlib_client.waitForResult(ros::Duration(timeout_value_));

    if (completion_handler) {

        if (!finished_before_timeout) {
            ROS_INFO_STREAM("Operation timed out.");
        }

        completion_handler(finished_before_timeout && actionlib_client.getState().isDone());
    }
}

void fluid::Client::requestOperation(
    std::string operation_identifier,
	geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    std::thread thread(&Client::waitForResult, this, operation_identifier, target_pose, completion_handler);
    thread.detach();
}
