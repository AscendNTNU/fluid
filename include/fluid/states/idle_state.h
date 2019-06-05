//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_IDLE_STATE_H
#define FLUID_FSM_IDLE_STATE_H

#include "state.h"

namespace fluid {

    /** \class IdleState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class IdleState: public State {

        const ros::Duration halt_interval_ = ros::Duration(1.0);        ///< The amoun of time to stay at idle before
                                                                        ///< saying we're finished executing this state.

        ros::Time initial_time_;                                        ///< The time the idle state starts up.
                                                                        ///< Makes it possible to halt at the 
                                                                        ///< idle state for an interval to 
                                                                        ///< make sure that we for example are stable
                                                                        ///< at ground after a land before we do 
                                                                        ///< something else.

    public:

        /**
         * Initializes the idle state.
         */
        explicit IdleState() : State(fluid::StateIdentifier::Idle, fluid::PX4::Offboard, true, false) {}

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

#endif //FLUID_FSM_IDLE_STATE_H
