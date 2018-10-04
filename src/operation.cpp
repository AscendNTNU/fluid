//
// Created by simengangstad on 04.10.18.
//

#include "../include/operation.h"
#include <iostream>

void Operation::stateBegan(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " began." << std::endl;
}

void Operation::stateFinished(const State &sender) {
    std::cout << "State " << typeid(sender).name() << " finshed." << std::endl;
}

void Operation::addState(std::shared_ptr<State> state_p) {
    state_p->state_delegate_p = shared_from_this();
    states_.push_back(state_p);
}

void Operation::completed(TransitionError transition_error) {
    std::cout << "Transition completed" << std::endl;
}

void Operation::perform() {

    // TODO: Dummy implementation, this should be based on an object graph
    for (int index = 0; index < states_.size(); index++) {
        auto state_p = states_[index];

        state_p->perform();

        if (index < states_.size() - 1) {
            Transition transition(state_p, states_[index + 1]);
            transition.transition_delegate_p = shared_from_this();
            transition.perform();
        }
    }
}