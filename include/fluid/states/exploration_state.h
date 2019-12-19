#ifndef FLUID_EXPLORATION_STATE_H
#define FLUID_EXPLORATION_STATE_H

#include "move_state.h"

namespace fluid {
    class ExplorationState : public MoveState {

        ExplorationState() : MoveState(1.0, 0.3, 1.0) {}
    }
}

#endif