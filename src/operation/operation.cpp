//
// Created by simengangstad on 04.10.18.
//

#include "../../include/operation/operation.h"
#include <iostream>

StateGraph Operation::state_graph;

void Operation::stateBegan(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " began." << std::endl;
}

void Operation::stateFinished(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " finshed." << std::endl;
}

void Operation::completed(TransitionError transition_error) {
    std::cout << "Transition completed" << std::endl;
}

void Operation::perform() {

    for (auto node: state_graph.getPlanToEndState("idle", "move")) {

        std::cout << node << std::endl;
    }
}
