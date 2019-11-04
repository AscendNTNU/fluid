#include "hold_state.h"

bool fluid::HoldState::hasFinishedExecution() const {
    return true;
}

void fluid::HoldState::initialize() {
    setpoint.x = getCurrentPose().pose.position.x;
    setpoint.y = getCurrentPose().pose.position.y;
    setpoint.z = getCurrentPose().pose.position.z;
    setpoint.type_mask = TypeMask::Position;
}