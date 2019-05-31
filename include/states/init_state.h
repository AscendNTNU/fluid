//
//  Created by Simen Gangstad on 15/10/2018.
//

#ifndef FLUID_FSM_INIT_STATE_H
#define FLUID_FSM_INIT_STATE_H

#include "../mavros/mavros_state.h"
#include "state_identifier.h"


namespace fluid {

    /** \class InitState
     *  \brief Makes sure everything is initialized (link to mavros and px4) before any further transitions are called.
     */
    class InitState: public MavrosState {

    private:

        bool initialized = false;                                 ///< Tells whether the drone is initialized or not.

    public:
        
        /** Initializes the init state.
         */
        explicit InitState() : MavrosState(fluid::StateIdentifier::Init, fluid::PX4::Offboard, false) {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;

        /**
         * Overridden function. @see State::perform
         */
        void perform(std::function<bool (void)> shouldAbort);
    };
}


#endif /* init_state_h */
