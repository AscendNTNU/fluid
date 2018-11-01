//
// Created by simengangstad on 04.10.18.
//

#include "../../../include/core/operation/operation.h"
#include <iostream>

fluid::StateGraph fluid::Operation::state_graph;

void fluid::Operation::perform(std::function<void (bool)> completion_handler) {

    if (!state_graph.isInitialized()) {
        state_graph.initialize();
    }

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(state_graph.current_state_p)) {
        completion_handler(false);
    }

    // Get plan to the destination state.
    std::list<std::shared_ptr<State>> plan = state_graph.getPlanToEndState(state_graph.current_state_p->identifier,
                                                                           destination_state_p_->identifier);


    std::shared_ptr<fluid::State> first_state_p = plan.front();
    plan.pop_front();

    fluid::Transition initial_transition(state_graph.getNodeHandlePtr(), first_state_p, plan.front(), 20);
    initial_transition.perform([]() {});

    for (const auto &state_p : plan) {
        state_p->perform();
        plan.pop_front();

        fluid::Transition transition(state_graph.getNodeHandlePtr(), state_p, plan.front(), 20);
        transition.perform([]() {});
        state_graph.current_state_p = plan.front();
    }

    fluid::Transition final_transition(state_graph.getNodeHandlePtr(), state_graph.current_state_p, final_state_p_, 20);
    final_transition.perform([]() {});

    completion_handler(true);
}
