#ifndef FLUID_FSM_HOLD_STATE_H
#define FLUID_FSM_HOLD_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class HoldState
     *  \brief Represents the state when the drone is hovering at a certain altitude
     */
    class HoldState: public State {

    public:

        explicit HoldState() : State(StateIdentifier::Hold, PX4::Offboard, true, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;
    };
}

#endif 
