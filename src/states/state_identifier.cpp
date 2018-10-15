//
//  state_identifier.cpp
//  fluid_fsm
//
//  Created by Simen Gangstad on 15/10/2018.
//  Copyright © 2018 Simen Gangstad. All rights reserved.
//

#include <ostream>
#include "../../include/states/state_identifier.h"

std::ostream& operator<<(std::ostream& os, const fluid::StateIdentifier identifier) {
    std::string description;
    
    switch (identifier) {
        case fluid::StateIdentifier::init:
            description = "init";
            break;
            
        case fluid::StateIdentifier::idle:
            description = "idle";
            break;
            
        case fluid::StateIdentifier::take_off:
            description = "take_off";
            break;
            
        case fluid::StateIdentifier::hold:
            description = "hold";
            break;
            
        case fluid::StateIdentifier::move:
            description = "move";
            break;
            
        case fluid::StateIdentifier::land:
            description = "land";
            break;
            
        default:
            description = "unknown";
            break;
    }
    
    return os << description;
}
