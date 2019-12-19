#include "land_state.h"

#include "core.h"

bool fluid::LandState::hasFinishedExecution() const {
    return getCurrentPose().pose.position.z - 0.0 < 0.05 && 
           std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::LandState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = 0.0;
    setpoint.type_mask = TypeMask::Position | TypeMask::IgnoreYaw;
}