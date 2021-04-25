/**
 * @file hold_operation.cpp
 */

#include "hold_operation.h"

#include "fluid.h"

HoldOperation::HoldOperation() : Operation(OperationIdentifier::HOLD, true, false) {}

bool HoldOperation::hasFinishedExecution() const {
    const float threshold = Fluid::getInstance().configuration.velocity_completion_threshold;
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < threshold;

    return low_enough_velocity;
}

void HoldOperation::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z + 0.1;  // Add a delta so the drone doesn't drop slightly
    setpoint.yaw = getCurrentYaw();
    setpoint.type_mask = TypeMask::POSITION;
}