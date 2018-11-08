//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/move_state.h"
#include "../../include/states/state_util.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::MoveState::hasFinishedExecution() {

    return StateUtil::distanceBetween(current_position_, position_target) < 0.2;
}

void fluid::MoveState::tick() {
    position_target.type_mask = fluid::DEFAULT_MASK;
}