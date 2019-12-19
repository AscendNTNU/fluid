#ifndef FLUID_TRAVELLING_STATE_H
#define FLUID_TRAVELLING_STATE_H

#include "move_state.h"

namespace fluid {
    class TravellingState : public MoveState {

        public: 
        TravellingState() : MoveState(20, 20, 22) {}
    };
}

#endif 