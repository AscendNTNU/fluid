#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up. This state's reference point is the current position,
     *         so the setpoint is irrelevant. 
     */
    class TakeOffState: public State {

    public:

        explicit TakeOffState() : State(StateIdentifier::TakeOff, PX4StateIdentifier::Offboard, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;
    };
}



#endif //FLUID_FSM_TAKE_OFF_STATE_H
