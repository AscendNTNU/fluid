#include "idle_state.h"

bool IdleState::hasFinishedExecution() const { return ros::Time::now() - initial_time_ > halt_interval_; }

void IdleState::initialize() {
    setpoint.position.x = setpoint.position.y = setpoint.position.z = 0.0;
    setpoint.type_mask = TypeMask::Idle;
}
