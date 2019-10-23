//
// Created by simengangstad on 11.10.18.
//

#include "idle_state.h"

bool fluid::IdleState::hasFinishedExecution() {

    return ros::Time::now() - initial_time_ > halt_interval_;
}

void fluid::IdleState::initialize() {
	idle_setpoint.x = 0.0;
	idle_setpoint.y = 0.0;
    idle_setpoint.z = 0.0;
}

std::vector<std::vector<double>> fluid::IdleState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) const {
    return fluid::Util::getSpineForSetpoint(idle_setpoint, idle_setpoint);
}

fluid::ControllerType fluid::IdleState::getPreferredController() {
    return ControllerType::Positional;
}