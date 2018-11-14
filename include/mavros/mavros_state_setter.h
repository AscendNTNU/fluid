//
// Created by simengangstad on 26.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_SETTER_H
#define FLUID_FSM_MAVROS_STATE_SETTER_H

#include <ros/ros.h>
#include "mavros_state_subscriber.h"
#include <mavros_msgs/State.h>

namespace fluid {
    /**
     * \class MavrosStateSetter
     * \brief Handles communication regarding setting states within the pixhawk.
     */
    class MavrosStateSetter {

    private:

        fluid::MavrosStateSubscriber state_subscriber_;                  ///< Grabs the current state

        ros::ServiceClient set_mode_client_;                             ///< Issues the state change commands

        const unsigned int update_interval_;                             ///< Amount of time passed before a new
                                                                         ///< set mode command is issued if the
                                                                         ///< state on the pixhawk doesn't match
                                                                         ///< the set state specified by mode.

        ros::Time last_request_ = ros::Time::now();                      ///< Keeps track of the time since the last
                                                                         ///< request.

        std::string mode_;                                               ///< Reference to the mode the state setter
                                                                         ///< should issue state changes for.

        mavros_msgs::SetMode set_mode_;                                  ///< Mavros mode reference, includes the ::mode


    public:


        /**
         * Sets ut the mavros state setter so it can issue state changes.
         *
         * @param node_handle_p         The node handle which issues the state changes.
         * @param message_queue_size    The message buffer size.
         * @param check_rate            The
         * @param mode                  The mode to change to.
         */
        MavrosStateSetter(const ros::NodeHandlePtr &node_handle_p,
                          unsigned int message_queue_size,
                          unsigned int update_interval,
                          const std::string &mode) :
        mode_(mode),
        update_interval_(update_interval),
        state_subscriber_(node_handle_p, message_queue_size),
        set_mode_client_(node_handle_p->serviceClient<mavros_msgs::SetMode>("mavros/set_mode")) {
            setMode(mode);
        }


        /**
         * Updates the mavros message set mode with a new mode.
         *
         * @param mode The new mode.
         */
        void setMode(std::string mode);


        /**
         * @return The current state the state subscriber has obtained.
         */
        mavros_msgs::State getCurrentState();

        /**
         * Checks if the current state is the state we want the Pixhawk to be in. If not, this function will issue
         * a state change command at a set interval (@see ::update_interval_) until it gets a response.
         *
         * @param completion_handler Called when the state change call is responded, will return with a flag whether
         *                           the state change succeeded or not.
         */
        void attemptToSetState(std::function<void (bool)> completion_handler);
    };
}

#endif //FLUID_FSM_MAVROS_STATE_SETTER_H
