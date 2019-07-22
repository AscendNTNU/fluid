//
// Created by simengangstad on 09.06.19
//

#ifndef FLUID_ROTATE_STATE_H
#define FLUID_ROTATE_STATE_H

#include "state.h"

namespace fluid {

    /** \class RotateState 
     *  \brief Represents the state where the is rotating at the current position.
     */
    class RotateState : public State {

    public:

        /** Initializes the rotate state.
         */
        explicit RotateState() : State(fluid::StateIdentifier::Rotate, fluid::PX4::Offboard, false, false) {}

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

#endif 
