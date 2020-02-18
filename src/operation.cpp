#include "operation.h"

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <utility>

#include "core.h"
#include "transition.h"
#include "util.h"

Operation::Operation(const StateIdentifier& destination_state_identifier, const std::vector<ascend_msgs::PositionYawTarget>& path)
    : destination_state_identifier(std::move(destination_state_identifier)), path(std::move(path)) {}

void Operation::perform(std::function<bool(void)> should_tick, std::function<void(bool)> completionHandler) {
    if (!validateOperationFromCurrentState(Core::getGraphPtr()->current_state_ptr)) {
        ROS_FATAL_STREAM("Operation to: "
                         << StateIdentifierStringMap.at(destination_state_identifier)
                         << " is not a valid operation from current state: "
                         << StateIdentifierStringMap.at(Core::getGraphPtr()->current_state_ptr->identifier));

        completionHandler(false);
        return;
    }

    // As the graph will return the shortest path in terms of states, we have to check if
    // the final setpoint is outside where we're currently at so we include a move state (if the move is on the path).
    geometry_msgs::Point current_position = Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position;
    current_position.z = 0;
    ascend_msgs::PositionYawTarget last_setpoint = path.back();
    last_setpoint.point.z = 0;

    float distanceToEndpoint = Util::distanceBetween(current_position, last_setpoint.point);
    bool shouldIncludeMove = (distanceToEndpoint >= Core::distance_completion_threshold) || path.size() > 1;

    // Get shortest path to the destination state from the current state. This will make it possible for
    // the FSM to transition to every state in order to get to the state we want to.
    std::vector<std::shared_ptr<State>> states = Core::getGraphPtr()->getPathToEndState(
        Core::getGraphPtr()->current_state_ptr->identifier, destination_state_identifier, shouldIncludeMove);

    if (states.empty()) {
        return;
    }

    for (auto state : states) {
        state->path = path;
    }

    ROS_INFO_STREAM("New operation requested to state: " << StateIdentifierStringMap.at(destination_state_identifier));

    std::stringstream stringstream;

    for (auto state : states) {
        stringstream << StateIdentifierStringMap.at(state->identifier) << " ";
    }

    ROS_INFO_STREAM("Will traverse through: " << stringstream.str() << "\n");

    // This will also only fire for operations that consist of more than one state (every operation other than init).
    // And in that case we transition to the next state in the path after the start state.
    if (Core::getGraphPtr()->current_state_ptr->identifier != destination_state_identifier) {
        transitionToState(states[1]);
    }

    for (int index = 0; index < states.size(); index++) {
        std::shared_ptr<State> state_p = states[index];

        ROS_INFO_STREAM("Executing state: " << StateIdentifierStringMap.at(state_p->identifier));
        Core::getStatusPublisherPtr()->status.current_state = StateIdentifierStringMap.at(state_p->identifier);
        Core::getStatusPublisherPtr()->publish();

        Core::getGraphPtr()->current_state_ptr = state_p;
        state_p->perform(should_tick, false);

        if (!should_tick()) {
            // We have to abort, so we transition to the steady state for the current state.
            std::shared_ptr<State> steady_state_ptr =
                Core::getGraphPtr()->getStateWithIdentifier(steady_state_map.at(state_p->identifier));
            ascend_msgs::PositionYawTarget current_pose;
            current_pose.point = state_p->getCurrentPose().pose.position;
            current_pose.yaw.data = 0; //Should be current yaw data
            steady_state_ptr->path = {current_pose};
            transitionToState(steady_state_ptr);
            completionHandler(false);

            return;
        }

        if (index < states.size() - 1) {
            transitionToState(states[index + 1]);
        }
    }

    std::shared_ptr<State> final_state_p = getFinalStatePtr();
    final_state_p->path = {path.back()};

    transitionToState(final_state_p);
    completionHandler(true);
}

void Operation::transitionToState(std::shared_ptr<State> state_ptr) const {
    Transition transition(Core::getGraphPtr()->current_state_ptr, state_ptr);
    transition.perform();
}

bool Operation::validateOperationFromCurrentState(std::shared_ptr<State> current_state_ptr) const {
    return Core::getGraphPtr()->areConnected(current_state_ptr->identifier, destination_state_identifier);
}

StateIdentifier Operation::getDestinationStateIdentifier() const { return destination_state_identifier; }

std::shared_ptr<State> Operation::getFinalStatePtr() const {
    return Core::getGraphPtr()->getStateWithIdentifier(steady_state_map.at(destination_state_identifier));
}

std::shared_ptr<State> Operation::getCurrentStatePtr() const { return Core::getGraphPtr()->current_state_ptr; }