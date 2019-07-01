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
    mavros_msgs::PositionTarget setpoint,
    std::function<void (bool)> completion_handler) {

    ActionlibClient actionlib_client(name_space + "/fluid_fsm_operation", false);

    actionlib_client.waitForServer();

    fluid::OperationGoal goal;
    goal.setpoint = setpoint;
    goal.destination_state_identifier.data = std::move(identifier);

    actionlib_client.sendGoal(goal);

    bool finished = actionlib_client.waitForResult();

    if (completion_handler) {
        completion_handler(actionlib_client.getState().isDone());
    }
}

void fluid::Client::requestTakeOff(double height, std::function<void (bool)> completion_handler) {
    // We pass in an empty pose here as the take off 
    // operation is relative, it will take off from
    // the current pose, so x and y setpoint is not relevant
    mavros_msgs::PositionTarget setpoint;
    setpoint.position.z = height;

    requestOperationToState(fluid::StateIdentifier::TakeOff, setpoint, completion_handler);    
}

void fluid::Client::requestTakeOff(std::function<void (bool)> completion_handler) {
    // We pass in 0 here as it will tell the state machine that we want to take off to the
    // default height specified from the parameters.
    requestTakeOff(0, completion_handler);
}

void fluid::Client::requestMove(mavros_msgs::PositionTarget setpoint, std::function<void (bool)> completion_handler) {
    requestOperationToState(fluid::StateIdentifier::Move, setpoint, completion_handler);
}

void fluid::Client::requestLand(std::function<void (bool)> completion_handler) {
    // We send in an empty pose here as the state machine will automatically land
    // at the current position.
    mavros_msgs::PositionTarget setpoint;
    requestOperationToState(fluid::StateIdentifier::Land, setpoint, completion_handler);
}

void fluid::Client::requestPositionFollow() {
    // We send in an empty pose here as the state machine will fetch the target
    // to follow. We also ignore the completion handler as the position follow
    // state will just keep on running until we request another state.
    mavros_msgs::PositionTarget setpoint;
    requestOperationToState(fluid::StateIdentifier::PositionFollow, setpoint, [](bool completed) {});
}

void fluid::Client::requestOperationToState(
    std::string identifier,
	mavros_msgs::PositionTarget setpoint,
    std::function<void (bool)> completion_handler) {

    std::thread thread(&Client::waitForResult, this, identifier, setpoint, completion_handler);
    thread.detach();
}
