/**
 * @file idle_state.cpp
 */

#include "idle_state.h"

bool IdleState::hasFinishedExecution() const { return ros::Time::now() - initial_time > HALT_INTERVAL; }

void IdleState::initialize() {
    setpoint.position.x = setpoint.position.y = setpoint.position.z = 0.0;
    setpoint.yaw = getCurrentYaw();
    setpoint.type_mask = TypeMask::Idle;
}
