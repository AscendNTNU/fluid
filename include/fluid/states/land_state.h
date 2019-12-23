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

        explicit LandState() : State(fluid::StateIdentifier::Land, fluid::PX4::Land, false, false) {}

        bool hasFinishedExecution() const override;
        void initialize() override;
   };
}

#endif 
