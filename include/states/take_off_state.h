//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "../mavros/mavros_state.h"
#include <ros/ros.h>

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up.
     */
    class TakeOffState: public MavrosState {
    public:

        /** Initializes the take off state with a pose.
         */
        explicit TakeOffState(ros::NodeHandlePtr node_handle_p) : MavrosState(node_handle_p, "take_off") {}

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



#endif //FLUID_FSM_TAKE_OFF_STATE_H
