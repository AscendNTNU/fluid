//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/hold_state.h"
#include "../../include/core/type_mask.h"

bool fluid::HoldState::hasFinishedExecution() {
    return false;
}

void fluid::HoldState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}