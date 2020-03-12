/**
 * @file land_state.cpp
 */

#include "land_state.h"

#include "core.h"

bool LandState::hasFinishedExecution() const {
    return getCurrentPose().pose.position.z - 0.0 < 0.05 &&
           std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;
}

void LandState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = 0.0;
    setpoint.yaw = getCurrentYaw();
    setpoint.type_mask = TypeMask::POSITION;
}