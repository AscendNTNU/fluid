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
    std::string identifier,
    geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    ActionlibClient actionlib_client(name_space + "/fluid_fsm_operation", false);

    actionlib_client.waitForServer();

    fluid::OperationGoal goal;
    goal.target_pose = target_pose;
    std_msgs::String type;
    type.data = std::move(identifier);
    goal.type = type;
    actionlib_client.sendGoal(goal);

    bool finished_before_timeout = actionlib_client.waitForResult(ros::Duration(timeout_value_));

    if (completion_handler) {
        completion_handler(finished_before_timeout && actionlib_client.getState().isDone());
    }
}

void fluid::Client::requestTakeOff(double height, std::function<void (bool)> completion_handler) {
    // We pass in an empty pose here as the take off 
    // operation is relative, it will take off from
    // the current pose, so x and y setpoint is not relevant
    geometry_msgs::Pose pose;
    pose.position.z = height;

    requestOperationToState(fluid::StateIdentifier::TakeOff, pose, completion_handler);    
}

void fluid::Client::requestTakeOff(std::function<void (bool)> completion_handler) {
    // We pass in 0 here as it will tell the state machine that we want to take off to the
    // default height specified from the parameters.
    requestTakeOff(0, completion_handler);
}

void fluid::Client::requestMove(geometry_msgs::Pose pose, std::function<void (bool)> completion_handler) {
    requestOperationToState(fluid::StateIdentifier::Move, pose, completion_handler);
}

void fluid::Client::requestLand(std::function<void (bool)> completion_handler) {
    // We send in an empty pose here as the state machine will automatically land
    // at the current position.
    geometry_msgs::Pose pose;
    requestOperationToState(fluid::StateIdentifier::Land, pose, completion_handler);
}

void fluid::Client::requestPositionFollow() {
    // We send in an empty pose here as the state machine will fetch the target
    // to follow. We also ignore the completion handler as the position follow
    // state will just keep on running until we request another state.
    geometry_msgs::Pose pose;
    requestOperationToState(fluid::StateIdentifier::PositionFollow, pose, [](bool completed) {});
}

void fluid::Client::requestOperationToState(
    std::string identifier,
	geometry_msgs::Pose target_pose,
    std::function<void (bool)> completion_handler) {

    std::thread thread(&Client::waitForResult, this, identifier, target_pose, completion_handler);
    thread.detach();
}
