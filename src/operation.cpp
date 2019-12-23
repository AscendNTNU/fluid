#include "operation.h"

#include <utility>
#include <algorithm>
#include <cmath>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "core.h"
#include "transition.h"
#include "util.h"

fluid::Operation::Operation(const std::string& destination_state_identifier,
                            const std::vector<geometry_msgs::Point>& path) :
                            destination_state_identifier_(std::move(destination_state_identifier)),
                            path(std::move(path)) {}

void fluid::Operation::perform(std::function<bool (void)> should_tick, std::function<void (bool)> completionHandler) {

    
    
    if (!validateOperationFromCurrentState(fluid::Core::getGraphPtr()->current_state_ptr)) {
        ROS_FATAL_STREAM("Operation to: " << 
                         destination_state_identifier_ << 
                         " is not a valid operation from current state: " <<
                         fluid::Core::getGraphPtr()->current_state_ptr->identifier);

        completionHandler(false);
        return;
    }

    // As the graph will return the shortest path in terms of states, we have to check if
    // the final setpoint is outside where we're currently at so we include a move state (if the move is on the path).
    geometry_msgs::Point current_position = fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position;
    current_position.z = 0;
    geometry_msgs::Point last_setpoint = path.back();
    last_setpoint.z = 0;

    float distanceToEndpoint = Util::distanceBetween(current_position, last_setpoint);
    bool shouldIncludeMove = (distanceToEndpoint >= fluid::Core::distance_completion_threshold) || path.size() > 1;


    // Get shortest path to the destination state from the current state. This will make it possible for
    // the FSM to transition to every state in order to get to the state we want to.
    std::vector<std::shared_ptr<State>> states = fluid::Core::getGraphPtr()->getPathToEndState(
                        fluid::Core::getGraphPtr()->current_state_ptr->identifier,
                        destination_state_identifier_, shouldIncludeMove);

    if (states.empty()) {
        return;
    }

    for (auto state : states) {
        state->path = path;
    }

    ROS_INFO_STREAM("New operation requested to state: " << destination_state_identifier_);

    std::stringstream stringstream;

    for (auto state : states) {
        stringstream << state->identifier << " ";
    }

    ROS_INFO_STREAM("Will traverse through: " << stringstream.str() << "\n");

    // This will also only fire for operations that consist of more than one state (every operation other than init).
    // And in that case we transition to the next state in the path after the start state.
    if (fluid::Core::getGraphPtr()->current_state_ptr->identifier != destination_state_identifier_) {
        transitionToState(states[1]);
    }

    for (int index = 0; index < states.size(); index++) {

        std::shared_ptr<fluid::State> state_p = states[index];

        ROS_INFO_STREAM("Executing state: " << state_p->identifier);
        fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
        fluid::Core::getStatusPublisherPtr()->publish();

        fluid::Core::getGraphPtr()->current_state_ptr = state_p;
        state_p->perform(should_tick, false);

        if (should_tick()) {

            // We have to abort, so we transition to the steady state for the current state.
            std::shared_ptr<fluid::State> steady_state_ptr = fluid::Core::getGraphPtr()->getStateWithIdentifier(steady_state_map_.at(state_p->identifier));
            steady_state_ptr->path = {state_p->getCurrentPose().pose.position};
            transitionToState(steady_state_ptr);
            completionHandler(false);

            return;
        }

        if (index < path.size() - 1) {
            transitionToState(states[index + 1]);
        }
    }

    std::shared_ptr<fluid::State> final_state_p = getFinalStatePtr();
    final_state_p->path = {path.back()};

    transitionToState(final_state_p);
    completionHandler(true);
}

void fluid::Operation::transitionToState(std::shared_ptr<fluid::State> state_p) {
    fluid::Transition transition(fluid::Core::getGraphPtr()->current_state_ptr, state_p);
    transition.perform();
}

bool fluid::Operation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const {
    return fluid::Core::getGraphPtr()->areConnected(current_state_ptr->identifier, destination_state_identifier_);
}

std::string fluid::Operation::getDestinationStateIdentifier() const {
    return destination_state_identifier_;
}

std::shared_ptr<fluid::State> fluid::Operation::getFinalStatePtr() const {
    return fluid::Core::getGraphPtr()->getStateWithIdentifier(steady_state_map_.at(destination_state_identifier_));
}

std::shared_ptr<fluid::State> fluid::Operation::getCurrentStatePtr() const {    
    return fluid::Core::getGraphPtr()->current_state_ptr;
}