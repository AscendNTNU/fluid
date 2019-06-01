//
// Created by simengangstad on 11.10.18.
//

#include "../../include/fluid/states/idle_state.h"
#include "../../include/fluid/core/type_mask.h"

bool fluid::IdleState::hasFinishedExecution() {
    return false;
}

void fluid::IdleState::tick() {
    setpoint.type_mask = fluid::TypeMask::Idle;
	setpoint.position.x = 0.0;
	setpoint.position.y = 0.0;
    setpoint.position.z = 0.0;
}