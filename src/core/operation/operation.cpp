//
// Created by simengangstad on 04.10.18.
//

#include "../../../include/core/operation/operation.h"
#include <iostream>
#include <core/operation/operation.h>

// TODO: Make refresh rate a paramter in the launch file

const int refresh_rate = 60;

fluid::Graph fluid::Operation::graph;

void fluid::Operation::perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) {

    if (!graph.isInitialized()) {
        graph.initialize();
    }

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(graph.current_state_p)) {
        ROS_ERROR("Not valid operation from current state!");
        completionHandler(false);
        return;
    }

    // Get plan to the destination state.
    std::vector<std::shared_ptr<State>> plan = graph.getPlanToEndState(graph.current_state_p->identifier,
                                                                       destination_state_identifier_);

    int startIndex = plan.size() > 1 ? 1 : 0;

    // This implicates that the plan's size is bigger than 1.
    if (graph.current_state_p->identifier != destination_state_identifier_) {
        fluid::Transition initial_transition(graph.getNodeHandlePtr(), plan[0], plan[1], refresh_rate);
        initial_transition.perform([]() {});
    }

    for (int index = startIndex; index < plan.size(); index++) {

        std::shared_ptr<fluid::State> state_p = plan[index];

        if (index == plan.size() - 1) {
            state_p->position_target = position_target;
        }

        state_p->perform(shouldAbort);

        if (shouldAbort()) {

            std::shared_ptr<fluid::State> final_state_p = graph.getStateWithIdentifier(final_state_identifier_);

            fluid::Transition final_transition(graph.getNodeHandlePtr(),
                                               graph.current_state_p,
                                               final_state_p,
                                               refresh_rate);
            final_transition.perform([]() {});

            // But if the current operation is the same as the next one, we shouldn't 
            // switch states.

            // TODO: Should set orientation here as well. But we need to convert between quaternions 
            //       and angles.
            final_state_p->position_target.position = graph.current_state_p->getCurrentPose().pose.position;
            graph.current_state_p = final_state_p;

            completionHandler(false);
            return;
        }

        graph.current_state_p = state_p;

        if (index < plan.size() - 1) {

            fluid::Transition transition(graph.getNodeHandlePtr(), state_p, plan[index + 1], refresh_rate);
            transition.perform([]() {});
        }
    }


    std::shared_ptr<fluid::State> final_state = graph.getStateWithIdentifier(final_state_identifier_);

    fluid::Transition final_transition(graph.getNodeHandlePtr(),
                                       graph.current_state_p,
                                       final_state,
                                       refresh_rate);
    final_transition.perform([]() {});

    graph.current_state_p = final_state;
    graph.current_state_p->position_target = position_target;

    completionHandler(true);
}

std::shared_ptr<fluid::State> fluid::Operation::getFinalStatePtr() {
    return graph.getStateWithIdentifier(final_state_identifier_);
}

std::shared_ptr<fluid::State> fluid::Operation::getCurrentStatePtr() {
    return graph.getStateWithIdentifier(graph.current_state_p->identifier);
}