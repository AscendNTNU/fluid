#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "state.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up. This state's reference point is the current position,
     *         so the setpoint is irrelevant. 
     */
    class TakeOffState: public State {
    public:

        /** Initializes the take off state.
         */
        explicit TakeOffState() : State(fluid::StateIdentifier::TakeOff, fluid::PX4::Offboard, false, true, true) {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::initialize
         */
        void initialize() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;
    };
}



#endif //FLUID_FSM_TAKE_OFF_STATE_H
