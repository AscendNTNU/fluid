//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "../core/state.h"
#include "state_identifier.h"

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing.
     */
    class LandState: public State {

    public:

        /**
         * Initializes the land state.
         */
        explicit LandState() : State(fluid::StateIdentifier::Land, fluid::PX4::Land, false) {}

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

#endif //FLUID_FSM_LAND_STATE_H
