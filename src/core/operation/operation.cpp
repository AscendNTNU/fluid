//
// Created by simengangstad on 04.10.18.
//

#include "../../include/operation/operation.h"
#include <iostream>

fluid::StateGraph fluid::Operation::state_graph;

void fluid::Operation::stateBegan(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " began." << std::endl;
}

void fluid::Operation::stateFinished(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " finshed." << std::endl;
}

void fluid::Operation::completed(TransitionError transition_error) {
    std::cout << "Transition completed" << std::endl;
}

void fluid::Operation::perform() {

    for (auto node: state_graph.getPlanToEndState("idle", "move")) {

        std::cout << node << std::endl;
    }
}
