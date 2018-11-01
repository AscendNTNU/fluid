//
// Created by simengangstad on 11.10.18.
//


#include "../../include/states/take_off_state.h"
#include "../../include/states/state_util.h"


bool fluid::TakeOffState::hasFinishedExecution() {
    return StateUtil::distanceBetween(current_position_, pose) < 0.05;
}

void fluid::TakeOffState::tick() {

}