//
// Created by simengangstad on 11.10.18.
//

#include "hold_state.h"

bool fluid::HoldState::hasFinishedExecution() {
    return false;
}

void fluid::HoldState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}