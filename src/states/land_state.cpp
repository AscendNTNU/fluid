//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::LandState::hasFinishedExecution() {
    return current_position_.pose.position.z < 0.35;
}

void fluid::LandState::tick() {
    position_target.type_mask = MASK_VELOCITY;
    position_target.velocity.z = -0.02;
}