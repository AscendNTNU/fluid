//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/move_state.h"
#include "../../include/tools/pose_util.h"
#include "../../include/mavros/type_mask.h"

bool fluid::MoveState::hasFinishedExecution() {
    return PoseUtil::distanceBetween(current_pose_, position_target) < 0.3;
}

void fluid::MoveState::tick() {
    position_target.type_mask = fluid::TypeMask::Default;
}