//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_IDLE_STATE_H
#define FLUID_FSM_IDLE_STATE_H

#include "../core/state.h"

namespace fluid {
    /** \class IdleState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class IdleState: public State {
    public:

        /** Initializes the idle state with a pose.
         *
         * @param pose The pose for the idle state.
         */
        IdleState(Pose pose) : State(fluid::StateIdentifier::idle, pose) {}

        /**
         * Performs the operation of setting the rotors to start spinning at ground
         */
        void perform();
    };
}

#endif //FLUID_FSM_IDLE_STATE_H
