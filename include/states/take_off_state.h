//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "../core/state.h"

namespace fluid {
    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up.
     */
    class TakeOffState: public State {
    public:

        /** Initializes the take off state with a pose.
         *
         * @param pose The pose for the take off state.
         */
        TakeOffState(Pose pose) : State("take_off", pose) {}

        /**
         * Performs the operation of taking off with the drone.
         */
        void perform();
    };
}



#endif //FLUID_FSM_TAKE_OFF_STATE_H
