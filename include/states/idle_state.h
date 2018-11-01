//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_IDLE_STATE_H
#define FLUID_FSM_IDLE_STATE_H

#include "../core/state.h"
#include "../mavros/mavros_state.h"

#include <ros/ros.h>

namespace fluid {

    /** \class IdleState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class IdleState: public MavrosState {
    public:

        /**
         * Initializes the idle state.
         */
        explicit IdleState(ros::NodeHandlePtr node_handle_p) : MavrosState(node_handle_p, "idle") {}

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

#endif //FLUID_FSM_IDLE_STATE_H
