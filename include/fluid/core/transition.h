//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_TRANSITION_H
#define FLUID_FSM_TRANSITION_H

#include <memory>
#include <ros/ros.h>

#include "mavros_state_link.h"
#include "state.h"

namespace fluid {
    /** \class Transition
     *  \breif Handles state changes and the communication with the FSM in PX4.
     */
    class Transition {
    private:

        fluid::MavrosStateLink mavros_state_link_;                             ///< Used to set states within the Pixhawk.

    public:
        const std::shared_ptr<fluid::State> source_state_p;                   ///< Source state of the transition

        const std::shared_ptr<fluid::State> destination_state_p;              ///< Destination state of the transition

        /**
         * Initializes a transition with source state and destination state.
         *
         * @param source_p The source state.
         * @param destination_p The destination state.
         */
        Transition(std::shared_ptr<State> source_state_p, std::shared_ptr<State> destination_state_p);

        /**
         * Performs the transition between the source state and the destination state.
         */
        void perform();
    };
}

#endif //FLUID_FSM_TRANSITION_H
