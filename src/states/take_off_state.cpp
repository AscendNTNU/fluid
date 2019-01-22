//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/take_off_state.h"
#include "../../include/tools/pose_util.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

bool fluid::TakeOffState::hasFinishedExecution() {
    return PoseUtil::distanceBetween(current_pose_, position_target) < 0.2;
}

void fluid::TakeOffState::tick() {
    position_target.type_mask = fluid::DEFAULT_MASK;
}