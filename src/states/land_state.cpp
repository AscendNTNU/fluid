//
// Created by simengangstad on 11.10.18.
//


#include "land_state.h"

bool fluid::LandState::hasFinishedExecution() {
    return current_pose_.pose.position.z - 0.0 < 0.05 && std::abs(getCurrentTwist().twist.linear.z) < 0.1;
}

void fluid::LandState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}