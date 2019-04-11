//
// Created by simengangstad on 04.10.18.
//

#include <utility>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include <assert.h>

#include "../../../include/core/operation/operation.h"
#include "../../../include/actionlib/operation_server.h"
#include "../../../include/core/core.h"

fluid::Operation::Operation(std::string identifier,
                            std::string destination_state_identifier,
                            std::string final_state_identifier,
                            mavros_msgs::PositionTarget position_target) :
                            identifier(std::move(identifier)),
                            destination_state_identifier_(std::move(destination_state_identifier)),
                            final_state_identifier_(std::move(final_state_identifier)),
                            position_target(std::move(position_target)) {}

void fluid::Operation::perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) {

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(fluid::Core::getGraphPtr()->current_state_p)) {
        ROS_FATAL_STREAM("Operation: " << identifier << "is not a valid operation from current state: " <<
                         fluid::Core::getGraphPtr()->current_state_p->identifier);
        completionHandler(false);
        return;
    }
    
    // Get shortest path to the destination state from the current state. This will make it possible for
    // the FSM to transition to every state in order to get to the state we want to.
    std::vector<std::shared_ptr<State>> path = fluid::Core::getGraphPtr()->getPathToEndState(
                        fluid::Core::getGraphPtr()->current_state_p->identifier,
                        destination_state_identifier_);

    if (path.empty()) {
        return;
    }


    // Since the path includes the current state we set the start index to the one after the current only if the path
    // doesn't consist of a single state (e.g. init operation). In that case we want to only run through that state. 
    int startIndex = path.size() > 1 ? 1 : 0;

    // This will also only fire for operations that consist of more than one state (every operation other than init).
    // And in that case we transition to the next state in the path after the start state.
    if (fluid::Core::getGraphPtr()->current_state_p->identifier != destination_state_identifier_) {
        transitionToState(path[1]);
    }

    for (int index = startIndex; index < path.size(); index++) {

        // TODO: What do we do here if the different states require different position targets?
        std::shared_ptr<fluid::State> state_p = path[index];
        
        if (index == path.size() - 1) {
            state_p->position_target = position_target;
        }

        fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
        fluid::Core::getStatusPublisherPtr()->publish();

        fluid::Core::getGraphPtr()->current_state_p = state_p;
        state_p->perform(shouldAbort);

        if (shouldAbort()) {

            // We have to abort, so we transition to the final state and set the current pose as the final state's
            // pose.
            std::shared_ptr<fluid::State> final_state_p = getFinalStatePtr();

            geometry_msgs::Quaternion poseQuat = state_p->getCurrentPose().pose.orientation;
            tf2::Quaternion tf2Quat(poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w);
            double roll = 0, pitch = 0, yaw = 0;
            tf2::Matrix3x3(tf2Quat).getRPY(roll, pitch, yaw);

            final_state_p->position_target.position = state_p->getCurrentPose().pose.position;
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            final_state_p->position_target.yaw = static_cast<float>(std::isnan(yaw) ? 0.0 : yaw);

            transitionToState(final_state_p);
            completionHandler(false);

            return;
        }

        if (index < path.size() - 1) {
            transitionToState(path[index + 1]);
        }
    }

    std::shared_ptr<fluid::State> final_state_p = getFinalStatePtr();
    final_state_p->position_target = position_target;

    transitionToState(final_state_p);
    completionHandler(true);
}

void fluid::Operation::transitionToState(std::shared_ptr<fluid::State> state_p) {
    fluid::Transition transition(fluid::Core::getGraphPtr()->current_state_p, state_p);
    transition.perform();
    fluid::Core::getGraphPtr()->current_state_p = state_p;
    fluid::Core::getStatusPublisherPtr()->status.current_state = state_p->identifier;
}


std::shared_ptr<fluid::State> fluid::Operation::getFinalStatePtr() {
    return fluid::Core::getGraphPtr()->getStateWithIdentifier(final_state_identifier_);
}

std::shared_ptr<fluid::State> fluid::Operation::getCurrentStatePtr() {    
    return fluid::Core::getGraphPtr()->current_state_p;
}