//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/take_off_state.h"
#include "../../include/states/state_util.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::TakeOffState::hasFinishedExecution() {
    return StateUtil::distanceBetween(current_position_, position_target) < 0.2;
}

void fluid::TakeOffState::tick() {
    position_target.type_mask = fluid::DEFAULT_MASK;
}