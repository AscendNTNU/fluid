//
// Created by simengangstad on 11.10.18.
//

#include "hold_state.h"

bool fluid::HoldState::hasFinishedExecution() {
    return true;
}

void fluid::HoldState::initialize() {
    initial_position.x = getCurrentPose().pose.position.x;
    initial_position.y = getCurrentPose().pose.position.y;
    initial_position.z = getCurrentPose().pose.position.z;
}

std::vector<std::vector<double>> fluid::HoldState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) const {
    return Util::getSpineForSetpoint(initial_position, initial_position);
}

fluid::ControllerType fluid::HoldState::getPreferredController() {
    return ControllerType::Positional;
}