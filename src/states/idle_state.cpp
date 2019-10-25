#include "idle_state.h"

bool fluid::IdleState::hasFinishedExecution() const {

    return ros::Time::now() - initial_time_ > halt_interval_;
}

void fluid::IdleState::initialize() {}

std::vector<ascend_msgs::Spline> fluid::IdleState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) {
    geometry_msgs::Point origin;

    return fluid::Util::getSplineForSetpoint(origin, origin);
}

fluid::ControllerType fluid::IdleState::getPreferredController() const {
    return ControllerType::Positional;
}