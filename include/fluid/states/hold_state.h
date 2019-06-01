//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_HOLD_STATE_H
#define FLUID_FSM_HOLD_STATE_H

#include "state.h"

namespace fluid {

    /** \class HoldState
     *  \brief Represents the state when the drone is hovering at a certain altitude
     */
    class HoldState: public State {

    public:

        /**
         * Initializes the hold state.
         */
        explicit HoldState() : State(fluid::StateIdentifier::Hold, fluid::PX4::Offboard, false) {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;
    };
}

#endif //FLUID_FSM_HOLD_STATE_H
