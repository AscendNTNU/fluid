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
    initial_position.x = getCurrentPose().pose.position.x;
    initial_position.y = getCurrentPose().pose.position.y;
	initial_position.z = path.front().z;

	if (initial_position.z <= 0.1) {
		initial_position.z = fluid::Core::default_height;
	}
}

std::vector<std::vector<double>> fluid::TakeOffState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) const {
    return Util::getSplineForSetpoint(initial_position, initial_position);
}

fluid::ControllerType fluid::TakeOffState::getPreferredController() const {
    return ControllerType::Positional; 
}
