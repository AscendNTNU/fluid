//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_IDLE_STATE_H
#define FLUID_FSM_IDLE_STATE_H

#include "../core/state.h"
#include "../mavros/mavros_state.h"

namespace fluid {

    /** \class IdleState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class IdleState: public MavrosState {
    public:

        /**
         * Initializes the idle state.
         */
        IdleState() : MavrosState("idle") {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution();

        /**
         * Overridden function. @see State::tick
         */
        void tick();
    };
}

#endif //FLUID_FSM_IDLE_STATE_H
