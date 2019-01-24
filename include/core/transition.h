#include <utility>

//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_TRANSITION_H
#define FLUID_FSM_TRANSITION_H

#include <memory>
#include <ros/ros.h>
#include "mavros/mavros_state_setter.h"
#include "state.h"

namespace fluid {
    /** \class Transition
     *  \breif Handles state changes and the communication with the FSM in PX4.
     */
    class Transition {
    private:
        fluid::MavrosStateSetter mavros_state_setter_;                        ///< Sets states within the Pixhawk.

        const unsigned int refresh_rate_;                                     ///< Refresh rate of the ros loop.

    public:
        const std::shared_ptr<fluid::State> source_state_p;                   ///< Source state of the transition

        const std::shared_ptr<fluid::State> destination_state_p;              ///< Destination state of the transition

        /**
         * Initializes a transition with source state and destination state.
         *
         * @param node_handle_p Used for initializing the mavros state setter.
         * @param source_p The source state.
         * @param destination_p The destination state.
         * @param refresh_rate The refresh rate the transition should operate at.
         */
        Transition(const ros::NodeHandlePtr &node_handle_p,
                   std::shared_ptr<State> source_state_p,
                   std::shared_ptr<State> destination_state_p,
                   unsigned int refresh_rate) :
                   mavros_state_setter_(node_handle_p, 1000, 1.0/static_cast<double>(refresh_rate), "OFFBOARD"),
                   source_state_p(std::move(source_state_p)),
                   destination_state_p(std::move(destination_state_p)),
                   refresh_rate_(refresh_rate) {}

        /**
         * Performs the transition between the source state and the destination state.
         */
        void perform();
    };
}

#endif //FLUID_FSM_TRANSITION_H
