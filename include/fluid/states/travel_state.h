#ifndef FLUID_TRAVEL_STATE_H
#define FLUID_TRAVEL_STATE_H

#include "move_state.h"
#include "state_identifier.h"

namespace fluid {
    class TravelState : public MoveState {

        public: 
            TravelState() : MoveState(StateIdentifier::Travel, 20, 10, 22) {}
    };
}

#endif 