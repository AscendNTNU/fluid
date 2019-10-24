#include "idle_state.h"

bool fluid::IdleState::hasFinishedExecution() {

    return ros::Time::now() - initial_time_ > halt_interval_;
}

void fluid::IdleState::initialize() {}

std::vector<std::vector<double>> fluid::IdleState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) const {
    geometry_msgs::Point origin;

    return fluid::Util::getSplineForSetpoint(origin, origin);
}

fluid::ControllerType fluid::IdleState::getPreferredController() {
    return ControllerType::Positional;
}