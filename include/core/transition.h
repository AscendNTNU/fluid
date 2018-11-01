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
    // TODO: Check the errors which can come from state changes in the FSM in PX4
    /** \enum TransitionErrorCode
     *  \brief Describes which kind of error which happened in a given transition.
     */
    enum TransitionErrorCode: int8_t {
        no_error = 0,
        px4_error = 1
    };



    /** \struct TransitionError
     *  \brief Describes the error which happened in the transitions between two states.
     */
    struct TransitionError {

        TransitionErrorCode error_code;                         ///< The code which specifies which transition
                                                                ///< error occurred

        std::string source_state_identifier;                    ///< The source state of the transition

        std::string destination_state_identifier;               ///< The destination state of the transition
    };



    /** \class Transition
     *  \breif Handles state changes and the communication with the FSM in PX4.
     */
    class Transition {
    private:
        ros::NodeHandle node_handle_;                                         ///< Handle for the mavros state setter

        fluid::MavrosStateSetter mavros_state_setter_;                        ///< Sets states within the Pixhawk.

        const unsigned int refresh_rate_;                                    ///< Refresh rate of the ros loop.

    public:
        const std::shared_ptr<State> source_state_p;                         ///< Source state of the transition

        const std::shared_ptr<State> destination_state_p;                    ///< Destination state of the transition

        /**
         * Initializes a transition with source state and destination state.
         *
         * @param source_p The source state.
         * @param destination_p The destination state.
         */
         // TODO: Gloablize message queue
        Transition(std::shared_ptr<State> source_state_p,
                   std::shared_ptr<State> destination_state_p,
                   unsigned int refresh_rate) :
                   mavros_state_setter_(node_handle_, 1000, refresh_rate, "OFFBOARD"),
                   source_state_p(source_state_p),
                   destination_state_p(destination_state_p),
                   refresh_rate_(refresh_rate) {}

        /**
         * Performs the transition between the source state and the destination state.
         *
         * @param completion_handler Fired when the state change completed
         */
        void perform(std::function<void (void)> completion_handler);
    };
}

#endif //FLUID_FSM_TRANSITION_H
