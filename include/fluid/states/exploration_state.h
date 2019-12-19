#ifndef FLUID_EXPLORATION_STATE_H
#define FLUID_EXPLORATION_STATE_H

#include "move_state.h"
#include "state_identifier.h"

namespace fluid {
    class ExplorationState : public MoveState {

        public:
            ExplorationState() : MoveState(fluid::StateIdentifier::Exploration, 1.0, 0.3, 1.0) {}
    };
}

#endif