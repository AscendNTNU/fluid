//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/land_state.h"
#include "../../include/mavros/type_mask.h"

bool fluid::LandState::hasFinishedExecution() {
    return land_detector_.hasLanded(position_target);
}

void fluid::LandState::tick() {
    position_target.type_mask = fluid::TypeMask::Default;
}