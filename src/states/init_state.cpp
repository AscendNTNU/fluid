//
//  Created by Simen Gangstad on 15/10/2018.
//


#include "../../include/states/init_state.h"
#include <iostream>

void fluid::InitState::perform() {
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateBegan(*this);
    }
    
    std::cout << "Initializing..." << std::endl;
    
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }
}
