//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "state.h"

namespace fluid {

    /** \class MoveState
     *  \brief Represents the state where the drone is moving from a to b.
     */
    class MoveState: public State {

    public:

        /** Initializes the move state.
         */
        explicit MoveState() : State(fluid::StateIdentifier::Move, fluid::PX4::Offboard, false, true) {}

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

#endif //FLUID_FSM_MOVE_STATE_H
