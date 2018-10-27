//
// Created by simengangstad on 26.10.18.
//

#include "../../include/operations/move_operation.h"
#include "../../include/core/state.h"
#include <memory>

// State delegate

void fluid::MoveOperation::stateBegan(const fluid::State &sender) {

}

void fluid::MoveOperation::stateFinished(const fluid::State &sender) {

}


// Transition delegate

void fluid::MoveOperation::completed(fluid::TransitionError transition_error) {

}


// Operation execution

void fluid::MoveOperation::perform(std::function<void (bool)> completion_handler) {

    switch (state_graph.current_state->identifier) {
        case fluid::StateIdentifier::init:
        case fluid::StateIdentifier::idle:
        case fluid::StateIdentifier::land:
        case fluid::StateIdentifier::take_off:

            completion_handler(false);
            return;
    }

    // Get plan
    std::vector<std::shared_ptr<State>> plan = state_graph.getPlanToEndState(state_graph.current_state->identifier,
                                                                             fluid::StateIdentifier::move);



    // When move state reports state finished we issue a state change to position hold and report that the operation
    // has finished


}

