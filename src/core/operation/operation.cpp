//
// Created by simengangstad on 04.10.18.
//

#include "../../../include/core/operation/operation.h"
#include <iostream>
#include <core/operation/operation.h>


fluid::StateGraph fluid::Operation::state_graph;

void fluid::Operation::perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) {

    if (!state_graph.isInitialized()) {
        state_graph.initialize();
    }

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(state_graph.current_state_p)) {
        completionHandler(false);
    }

    // Get plan to the destination state.
    std::vector<std::shared_ptr<State>> plan = state_graph.getPlanToEndState(state_graph.current_state_p->identifier,
                                                                             destination_state_identifier_);

    int startIndex = plan.size() > 1 ? 1 : 0;

    // This implicates that the plan's size is bigger than 1.
    if (state_graph.current_state_p->identifier != destination_state_identifier_) {
        fluid::Transition initial_transition(state_graph.getNodeHandlePtr(), plan[0], plan[1], 20);
        initial_transition.perform([]() {});
    }

    for (int index = startIndex; index < plan.size(); index++) {

        std::shared_ptr<fluid::State> state_p = plan[index];

        if (index == plan.size() - 1) {
            state_p->position_target = position_target;
        }

        state_p->perform(shouldAbort);

        if (shouldAbort()) {
            fluid::Transition final_transition(state_graph.getNodeHandlePtr(),
                                               state_graph.current_state_p,
                                               state_graph.getStateWithIdentifier(final_state_identifier_),
                                               20);
            final_transition.perform([]() {});

            completionHandler(false);
            return;
        }

        state_graph.current_state_p = state_p;

        if (index < plan.size() - 1) {

            fluid::Transition transition(state_graph.getNodeHandlePtr(), state_p, plan[index + 1], 20);
            transition.perform([]() {});
        }
    }


    std::shared_ptr<fluid::State> final_state = state_graph.getStateWithIdentifier(final_state_identifier_);

    fluid::Transition final_transition(state_graph.getNodeHandlePtr(),
                                       state_graph.current_state_p,
                                       final_state,
                                       20);
    final_transition.perform([]() {});

    state_graph.current_state_p = final_state;
    state_graph.current_state_p->position_target = position_target;

    completionHandler(true);
}

std::shared_ptr<fluid::State> fluid::Operation::getFinalStatePtr() {
    return state_graph.getStateWithIdentifier(final_state_identifier_);
}
