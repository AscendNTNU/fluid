//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/idle_state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::IdleState::hasFinishedExecution() {
    return false;
}

void fluid::IdleState::tick() {
	position_target.type_mask = fluid::IDLE_MASK;
	position_target.position.x = 0.0;
	position_target.position.y = 0.0;
    position_target.position.z = 0.0;
}