//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/idle_state.h"


bool fluid::IdleState::hasFinishedExecution() {
    return false;
}

void fluid::IdleState::tick() {
    position_target.position.z = 0.0;
}