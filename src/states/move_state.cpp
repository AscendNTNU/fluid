//
// Created by simengangstad on 11.10.18.
//

#include "../../include/states/move_state.h"
#include "../../include/states/state_util.h"

bool fluid::MoveState::hasFinishedExecution() {
    return StateUtil::distanceBetween(current_position_, pose) < 0.05;
}

void fluid::MoveState::tick() {

}