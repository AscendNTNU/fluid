//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include "../../include/mavros/type_mask.h"

bool fluid::LandState::hasFinishedExecution() {
    return current_pose_.pose.position.z - 0.0 < 0.05 && std::abs(getCurrentTwist().twist.linear.z) < 0.1;
}

void fluid::LandState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}