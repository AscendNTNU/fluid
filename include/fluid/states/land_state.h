#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing. This state happens from the current position.
     */
    class LandState: public State {

    public:

        explicit LandState() : State(StateIdentifier::Land, PX4StateIdentifier::Land, false, false) {}

        bool hasFinishedExecution() const override;
        void initialize() override;
   };
}

#endif 
