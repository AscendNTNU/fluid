//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "../core/state.h"

namespace fluid {
    /** \class LandState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class LandState: public State {
    public:

        /** Initializes the land state with a pose.
         *
         * @param pose The pose for the land state.
         */
        LandState(Pose pose) : State("land", pose) {}

        /**
         * Performs the operation of landing the drone.
         */
        void perform();
    };
}

#endif //FLUID_FSM_LAND_STATE_H
