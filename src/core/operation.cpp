//
// Created by simengangstad on 04.10.18.
//

#include "operation.h"

#include <utility>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "core.h"
#include "transition.h"
#include "pose_util.h"

fluid::Operation::Operation(std::string destination_state_identifier,
                            mavros_msgs::PositionTarget position_target) :
                            destination_state_identifier_(std::move(destination_state_identifier)),
                            position_target(std::move(position_target)) {}

void fluid::Operation::perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) {

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(fluid::Core::getGraphPtr()->current_state_ptr)) {
        ROS_FATAL_STREAM("Operation to: " << destination_state_identifier_ << "is not a valid operation from current state: " <<
                         fluid::Core::getGraphPtr()->current_state_ptr->identifier);
        completionHandler(false);
        return;
    }

    // As the graph will return the shortest path in terms of states, we have to check if
    // the setpoint is outside where we're currently at so we include a move state (if the move is on the path).

    bool shouldIncludeMove = PoseUtil::distanceBetween(fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose(), position_target) >= fluid::Core::distance_completion_threshold;

    // Get shortest path to the destination state from the current state. This will make it possible for
    // the FSM to transition to every state in order to get to the state we want to.
    std::vector<std::shared_ptr<State>> path = fluid::Core::getGraphPtr()->getPathToEndState(
                        fluid::Core::getGraphPtr()->current_state_ptr->identifier,
                        destination_state_identifier_, shouldIncludeMove);

    if (path.empty()) {
        return;
    }

    // This will also only fire for operations that consist of more than one state (every operation other than init).
    // And in that case we transition to the next state in the path after the start state.
    if (fluid::Core::getGraphPtr()->current_state_ptr->identifier != destination_state_identifier_) {
        transitionToState(path[1]);
    }

    for (int index = 0; index < path.size(); index++) {

        std::shared_ptr<fluid::State> state_p = path[index];
        
        state_p->setpoint = position_target;
        
        fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
        fluid::Core::getStatusPublisherPtr()->publish();

        fluid::Core::getGraphPtr()->current_state_ptr = state_p;
        state_p->perform(shouldAbort, false);

        if (shouldAbort()) {

            // We have to abort, so we transition to the steady state for the current state.
            std::shared_ptr<fluid::State> steady_state_ptr = fluid::Core::getGraphPtr()->getStateWithIdentifier(steady_state_map_.at(state_p->identifier));

            geometry_msgs::Quaternion poseQuat = state_p->getCurrentPose().pose.orientation;
            tf2::Quaternion tf2Quat(poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w);
            double roll = 0, pitch = 0, yaw = 0;
            tf2::Matrix3x3(tf2Quat).getRPY(roll, pitch, yaw);

            steady_state_ptr->setpoint.position = state_p->getCurrentPose().pose.position;
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            steady_state_ptr->setpoint.yaw = static_cast<float>(std::isnan(yaw) ? 0.0 : yaw);

            transitionToState(steady_state_ptr);
            completionHandler(false);

            return;
        }

        if (index < path.size() - 1) {
            transitionToState(path[index + 1]);
        }
    }

    std::shared_ptr<fluid::State> final_state_p = getFinalStatePtr();
    final_state_p->setpoint = position_target;

    transitionToState(final_state_p);
    completionHandler(true);
}

void fluid::Operation::transitionToState(std::shared_ptr<fluid::State> state_p) {
    fluid::Transition transition(fluid::Core::getGraphPtr()->current_state_ptr, state_p);
    transition.perform();
    fluid::Core::getGraphPtr()->current_state_ptr = state_p;
    fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
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