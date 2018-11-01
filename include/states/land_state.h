//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_LAND_STATE_H
#define FLUID_FSM_LAND_STATE_H

#include "../mavros/mavros_state.h"

#include <ros/ros.h>

namespace fluid {

    /** \class LandState
     *  \brief Represents the state where the drone is landing.
     */
    class LandState: public MavrosState {
    public:

        /**
         * Initializes the land state.
         */
        explicit LandState(ros::NodeHandlePtr node_handle_p) : MavrosState(node_handle_p, "land") {}

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
