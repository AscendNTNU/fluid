#ifndef FLUID_TRAVELLING_STATE_H
#define FLUID_TRAVELLING_STATE_H

#include "move_state.h"
#include "state_identifier.h"

namespace fluid {
    class TravellingState : public MoveState {

        public: 
            TravellingState() : MoveState(StateIdentifier::Travelling, 20, 10, 22) {}
    };
}

#endif 