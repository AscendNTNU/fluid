//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/idle_state.h"
#include <iostream>

void fluid::IdleState::perform() {
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateBegan(*this);
    }

    std::cout << "Idle..." << std::endl;

    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }
}
