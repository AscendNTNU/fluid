#include "take_off_state.h"
#include "util.h"
#include "core.h"

bool fluid::TakeOffState::hasFinishedExecution() const {
    return Util::distanceBetween(getCurrentPose().pose.position, initial_position) < 0.1 &&
    	   std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::TakeOffState::initialize() {
    setpoint.x = getCurrentPose().pose.position.x;
    setpoint.y = getCurrentPose().pose.position.y;
	setpoint.z = Core::default_height;
    setpoint.type_mask = TypeMask::Position;
}
