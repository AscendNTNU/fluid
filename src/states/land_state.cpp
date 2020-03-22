/**
 * @file land_state.cpp
 */

#include "land_state.h"

#include "core.h"

LandState::LandState() : State(StateIdentifier::LAND, true) {}

bool LandState::isBelowThreshold() const {
    return getCurrentPose().pose.position.z < 0.05 &&
           std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;
}

bool LandState::hasFinishedExecution() const { return isBelowThreshold() && setpoint.type_mask == TypeMask::IDLE; }

void LandState::initialize() {
    // If land is issued and the drone is currently at ground, just keep sending setpoints with idle type mask
    if (isBelowThreshold()) {
        setpoint.position.x = setpoint.position.y = setpoint.position.z = 0;
        setpoint.type_mask = TypeMask::IDLE;
    } else {
        setpoint.position.x = getCurrentPose().pose.position.x;
        setpoint.position.y = getCurrentPose().pose.position.y;
        setpoint.position.z = 0.0;
        setpoint.yaw = getCurrentYaw();
        setpoint.type_mask = TypeMask::POSITION;
    }
}

void LandState::tick() {
    if (isBelowThreshold()) {
        setpoint.position.x = setpoint.position.y = setpoint.position.z = 0;
        setpoint.type_mask = TypeMask::IDLE;
    }
}