//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "../mavros/mavros_state.h"

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing.
     */
    class LandState: public MavrosState {
    public:

        /**
         * Initializes the land state.
         */
        LandState() : MavrosState("land") {}

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

#endif //FLUID_FSM_LAND_STATE_H
