#include "land_state.h"

#include "core.h"

bool fluid::LandState::hasFinishedExecution() const {
    return getCurrentPose().pose.position.z - 0.0 < 0.05 && 
           std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;
}

void fluid::LandState::initialize() {
    initial_position.x = getCurrentPose().pose.position.x;
    initial_position.y = getCurrentPose().pose.position.y;
    initial_position.z = 0.0;
}

std::vector<ascend_msgs::Spline> fluid::LandState::getSplinesForPath(const std::vector<geometry_msgs::Point>& path) {
    return Util::getSplineForSetpoint(initial_position, initial_position);
}

fluid::ControllerType fluid::LandState::getPreferredController() const {
    return ControllerType::Positional;
}