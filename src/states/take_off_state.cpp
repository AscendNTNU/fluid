#include "take_off_state.h"
#include "util.h"
#include "core.h"

bool fluid::TakeOffState::hasFinishedExecution() {
    return Util::distanceBetween(current_pose_.pose.position, setpoint.position) < 0.1 &&
    	   std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::TakeOffState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
	setpoint.position.z = path.front().z;

	if (setpoint.position.z <= 0.1) {
		setpoint.position.z = fluid::Core::default_height;
	}
}

void fluid::TakeOffState::tick() {
    setpoint.type_mask = fluid::TypeMask::Position;
}
