//
// Created by simengangstad on 11.10.18.
//


#include "../../include/fluid/states/take_off_state.h"
#include "../../include/fluid/tools/pose_util.h"
#include "../../include/fluid/core/type_mask.h"

bool fluid::TakeOffState::hasFinishedExecution() {
    return PoseUtil::distanceBetween(current_pose_, setpoint) < 0.3 && 
    	   std::abs(getCurrentTwist().twist.linear.x) < 0.05 && 
    	   std::abs(getCurrentTwist().twist.linear.y) < 0.05 && 
    	   std::abs(getCurrentTwist().twist.linear.z) < 0.05;
}

void fluid::TakeOffState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}