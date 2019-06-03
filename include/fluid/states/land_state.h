//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "state.h"

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing. This state happens from the current position.
     */
    class LandState: public State {

    public:

        /**
         * Initializes the land state.
         */
        explicit LandState() : State(fluid::StateIdentifier::Land, fluid::PX4::Land, false, false) {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::init
         */
        void initialize() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;
    };
}

#endif //FLUID_FSM_LAND_STATE_H
