//
// Created by simengangstad on 04.10.18.
//

#include "../../../include/core/operation/operation.h"
#include <iostream>

fluid::StateGraph fluid::Operation::state_graph;

void fluid::Operation::perform(std::function<void (bool)> completion_handler) {

    // Check if it makes sense to carry out this operation given the current state.
    if (!validateOperationFromCurrentState(state_graph.current_state)) {
        completion_handler(false);
    }

    // Get plan to the destination state.
    std::vector<std::shared_ptr<State>> plan = state_graph.getPlanToEndState(state_graph.current_state->identifier,
                                                                             destination_state_p_->identifier);

    // When move state reports state finished we issue a state change to position hold and report that the operation
    // has finished



    completion_handler(true);
}
