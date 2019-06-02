//
// Created by simengangstad on 11.10.18.
//


#include "take_off_state.h"
#include "pose_util.h"
#include "core.h"
bool fluid::TakeOffState::hasFinishedExecution() {
    return PoseUtil::distanceBetween(current_pose_, setpoint) < fluid::Core::distance_completion_threshold &&
    	   std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::TakeOffState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;

	if (setpoint.position.z <= 0.1) {
		setpoint.position.z = fluid::Core::default_height;
	}
}

void fluid::TakeOffState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}