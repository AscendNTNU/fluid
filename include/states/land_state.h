//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "../mavros/mavros_state.h"
#include "state_identifier.h"
#include "../tools/land_detector.h"

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing.
     */
    class LandState: public MavrosState {

    private:

        fluid::LandDetector land_detector_;                 ///< Checks when the drone has landed.

    public:

        /**
         * Initializes the land state.
         */
        explicit LandState() : MavrosState(fluid::StateIdentifier::Land, fluid::PX4::Land), land_detector_() {}

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

#endif //FLUID_FSM_LAND_STATE_H
