//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::LandState::hasFinishedExecution() {
    return land_detector_.hasLanded(position_target);
}

void fluid::LandState::tick() {
	position_target.type_mask = DEFAULT_MASK;
	// position_target.type_mask = MASK_VELOCITY & ~(IGNORE_PX | IGNORE_PY);  
    // position_target.velocity.z = -0.2;
}