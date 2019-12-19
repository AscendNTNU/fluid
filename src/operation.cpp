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

void fluid::Operation::perform(std::function<bool (void)> tick, std::function<void (bool)> completionHandler) {

    if (!validateOperationFromCurrentState(fluid::Core::getGraphPtr()->current_state_ptr)) {
        ROS_FATAL_STREAM("Operation to: " << 
                         destination_state_identifier_ << 
                         "is not a valid operation from current state: " <<
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

    // We can do this as most states are relative to the current position.
    for (auto state : states) {
        state->path = path;
    }

    float yaw = 0;

    // If we have to move, we want the rotate state to rotate in that direction and all succeeding states to also be in
    // that yaw.
    if (shouldIncludeMove) {
        float dx = path.front().x - fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position.x;
        float dy = path.front().y - fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position.y;

        yaw = atan2(dy, dx);

        // Go through states after rotate and append the yaw to all of them so we don't snap back to the 
        // setpoint yaw

        // Retrieve the iterator for rotate in the path
        auto iterator = std::find_if(states.begin(), states.end(), [] (const std::shared_ptr<fluid::State>& state) -> bool {
            return state->identifier == fluid::StateIdentifier::Rotate;
        });

        // Go through all states after rotate and set the yaw pointing in the direction we want to move.
        /*while (iterator != path.end()) {
            (*iterator)->setpoint.yaw = yaw;
            iterator++;
        }*/
    }


    // This will also only fire for operations that consist of more than one state (every operation other than init).
    // And in that case we transition to the next state in the path after the start state.
    if (fluid::Core::getGraphPtr()->current_state_ptr->identifier != destination_state_identifier_) {
        transitionToState(states[1]);
    }

    for (int index = 0; index < states.size(); index++) {

        std::shared_ptr<fluid::State> state_p = states[index];
        
        fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
        fluid::Core::getStatusPublisherPtr()->publish();

        fluid::Core::getGraphPtr()->current_state_ptr = state_p;
        state_p->perform(tick, false);

        if (!tick()) {

            // We have to abort, so we transition to the steady state for the current state.
            std::shared_ptr<fluid::State> steady_state_ptr = fluid::Core::getGraphPtr()->getStateWithIdentifier(steady_state_map_.at(state_p->identifier));

            geometry_msgs::Quaternion poseQuat = state_p->getCurrentPose().pose.orientation;
            tf2::Quaternion tf2Quat(poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w);
            double roll = 0, pitch = 0, yaw = 0;
            tf2::Matrix3x3(tf2Quat).getRPY(roll, pitch, yaw);

            steady_state_ptr->path = {state_p->getCurrentPose().pose.position};
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            // steady_state_ptr->setpoint.yaw = static_cast<float>(std::isnan(yaw) ? 0.0 : yaw);

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
    // final_state_p->setpoint.yaw = yaw;

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