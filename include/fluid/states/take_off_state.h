//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "../core/state.h"
#include "state_identifier.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up.
     */
    class TakeOffState: public State {
    public:

        /** Initializes the take off state.
         */
        explicit TakeOffState() : State(fluid::StateIdentifier::TakeOff, fluid::PX4::Offboard, true) {}

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



#endif //FLUID_FSM_TAKE_OFF_STATE_H
