#include "idle_state.h"

bool fluid::IdleState::hasFinishedExecution() const {

    return ros::Time::now() - initial_time_ > halt_interval_;
}

void fluid::IdleState::initialize() {
    setpoint.x = setpoint.y = setpoint.z = 0.0;
    setpoint.type_mask = TypeMask::Idle;

}
