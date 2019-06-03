//
// Created by simengangstad on 11.10.18.
//

#include "hold_state.h"

bool fluid::HoldState::hasFinishedExecution() {
    return true;
}

void fluid::HoldState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z;
}


void fluid::HoldState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}