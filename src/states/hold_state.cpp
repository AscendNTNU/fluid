/**
 * @file hold_state.cpp
 */

#include "hold_state.h"

#include "core.h"

HoldState::HoldState() : State(StateIdentifier::HOLD, true) {}

bool HoldState::hasFinishedExecution() const {
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < Core::velocity_completion_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < Core::velocity_completion_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;

    return low_enough_velocity;
}

void HoldState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z + 0.1;  // Add a delta so the drone doesn't drop slightly
    setpoint.yaw = getCurrentYaw();
    setpoint.type_mask = TypeMask::POSITION;
}