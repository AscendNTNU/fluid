//
// Created by simengangstad on 04.10.18.
//

#include "../../../include/core/operation/operation.h"
#include <core/operation/operation.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

fluid::StateGraph fluid::Operation::graph;

void fluid::Operation::perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) {

    // The graph needs to be set up before use (loaded with the given states).
    if (!graph.isConfigured()) {
        graph.configure(refresh_rate_);
    }

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(graph.current_state_p)) {
        ROS_ERROR("Not valid operation from current state!");
        completionHandler(false);
        return;
    }

    // Get shortest path to the destination state from the current state. This will make it possible for
    // the FSM to transition to every state in order to get to the state we want to.
    std::vector<std::shared_ptr<State>> path = graph.getPathToEndState(graph.current_state_p->identifier,
                                                                       destination_state_identifier_);

    if (path.size() == 0) {
        return;
    }

    int startIndex = path.size() > 1 ? 1 : 0;

    // This implicates that the plan's size is bigger than 1.
    if (graph.current_state_p->identifier != destination_state_identifier_) {
        transitionToState(path[1]);
    }

    for (int index = startIndex; index < path.size(); index++) {
        // TODO: What do we do here if the different states require different position targets?

        std::shared_ptr<fluid::State> state_p = path[index];

        if (index == path.size() - 1) {
            state_p->position_target = position_target;
        }

        graph.current_state_p = state_p;
        state_p->perform(shouldAbort);

        if (shouldAbort()) {

            std::shared_ptr<fluid::State> final_state_p = getFinalStatePtr();

            geometry_msgs::Quaternion poseQuat = state_p->getCurrentPose().pose.orientation;
            tf2::Quaternion tf2Quat(poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2Quat).getRPY(roll, pitch, yaw);
    
            final_state_p->position_target.position = state_p->getCurrentPose().pose.position;
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            final_state_p->position_target.yaw = std::isnan(yaw) ? 0.0 : yaw;   

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

    fluid::Transition transition(graph.getNodeHandlePtr(), graph.current_state_p, state_p, refresh_rate_);
    transition.perform();
    graph.current_state_p = state_p;
}


std::shared_ptr<fluid::State> fluid::Operation::getFinalStatePtr() {
    return graph.getStateWithIdentifier(final_state_identifier_);
}

std::shared_ptr<fluid::State> fluid::Operation::getCurrentStatePtr() {
    return graph.getStateWithIdentifier(graph.current_state_p->identifier);
}