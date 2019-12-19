#include "hold_state.h"

bool fluid::HoldState::hasFinishedExecution() const {
    return true;
}

void fluid::HoldState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z;
    setpoint.type_mask = TypeMask::Position | TypeMask::IgnoreYaw;
}