/**
 * @file land_operation.cpp
 */

#include "land_operation.h"

#include "fluid.h"

LandOperation::LandOperation() : Operation(OperationIdentifier::LAND, true, true) {}

bool LandOperation::isBelowThreshold() const {
    return getCurrentPose().pose.position.z < 0.05 &&
           std::abs(getCurrentTwist().twist.linear.z) <
               Fluid::getInstance().configuration.velocity_completion_threshold;
}

bool LandOperation::hasFinishedExecution() const { return isBelowThreshold() && setpoint.type_mask == TypeMask::IDLE; }

void LandOperation::initialize() {
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

void LandOperation::tick() {
    if (isBelowThreshold()) {
        setpoint.position.x = setpoint.position.y = setpoint.position.z = 0;
        setpoint.type_mask = TypeMask::IDLE;
    }
}