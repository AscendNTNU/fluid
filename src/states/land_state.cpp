//
// Created by simengangstad on 11.10.18.
//


#include "land_state.h"

bool fluid::LandState::hasFinishedExecution() {
    return current_pose_.pose.position.z - 0.0 < 0.05 && 
           std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::LandState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = 0.0;
}

void fluid::LandState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}