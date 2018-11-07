//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::LandState::hasFinishedExecution() {
    ROS_INFO_STREAM(current_position_.pose.position.z);
    return current_position_.pose.position.z < 0.02;
}

void fluid::LandState::tick() {
    position_target.type_mask = MASK_VELOCITY;
    position_target.velocity.z = -0.02;
}