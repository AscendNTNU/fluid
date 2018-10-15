//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include <iostream>

void fluid::LandState::perform() {
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateBegan(*this);
    }

    std::cout << "Landing..." << std::endl;

    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }
}
