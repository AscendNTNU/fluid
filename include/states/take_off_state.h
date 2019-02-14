//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "../mavros/mavros_state.h"
#include "state_defines.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up.
     */
    class TakeOffState: public MavrosState {
    public:

        /** Initializes the take off state.
         */
        explicit TakeOffState() : MavrosState(fluid::StateIdentifiers::TAKE_OFF, fluid::PX4::OFFBOARD) {}

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
