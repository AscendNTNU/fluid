#include "take_off_state.h"
#include "core.h"
#include "util.h"

bool TakeOffState::hasFinishedExecution() const {
    return Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < 0.1 &&
           std::abs(getCurrentTwist().twist.linear.x) < Core::velocity_completion_threshold &&
           std::abs(getCurrentTwist().twist.linear.y) < Core::velocity_completion_threshold &&
           std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;
}

void TakeOffState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    if (path.size() == 0) { setpoint.position.z = Core::default_height; }
    else { setpoint.position.z = path[0].point.z; }
    setpoint.type_mask = TypeMask::Position | TypeMask::IgnoreYaw;
}
