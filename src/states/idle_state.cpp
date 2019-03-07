//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/idle_state.h"
#include "../../include/mavros/type_mask.h"

bool fluid::IdleState::hasFinishedExecution() {
    return false;
}

void fluid::IdleState::tick() {
    position_target.type_mask = fluid::TypeMask::Idle;
	position_target.position.x = 0.0;
	position_target.position.y = 0.0;
    position_target.position.z = 0.0;
}