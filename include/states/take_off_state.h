//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "../mavros/mavros_state.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up.
     */
    class TakeOffState: public MavrosState {
    public:

        /** Initializes the take off state with a pose.
         */
        TakeOffState() : MavrosState("take_off") {}

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



#endif //FLUID_FSM_TAKE_OFF_STATE_H
