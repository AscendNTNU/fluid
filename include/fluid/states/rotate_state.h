#ifndef FLUID_ROTATE_STATE_H
#define FLUID_ROTATE_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class RotateState 
     *  \brief Represents the state where the is rotating at the current position.
     */
    class RotateState : public State {

    public:

        explicit RotateState() : State(fluid::StateIdentifier::Rotate, fluid::PX4::Offboard, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;
    };
}

#endif 
