//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/take_off_state.h"
#include <iostream>

void fluid::TakeOffState::perform() {
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateBegan(*this);
    }

    std::cout << "Taking off..." << std::endl;

    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }
}
