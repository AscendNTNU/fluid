//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/hold_state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::HoldState::hasFinishedExecution() {
    return false;
}

void fluid::HoldState::tick() {
    position_target.type_mask = fluid::DEFAULT_MASK;
}