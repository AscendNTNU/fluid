//
// Created by simengangstad on 26.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_LINK_H
#define FLUID_FSM_MAVROS_STATE_LINK_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

namespace fluid {
    /**
     * \class MavrosStateLink
     * \brief Handles communication regarding setting and retrieving states within the pixhawk.
     */
    class MavrosStateLink {

    private:

        ros::NodeHandle node_handle_;                                    ///< Used to set up the set mode client and the subscriber.

        ros::Subscriber state_subscriber_;                               ///< Retrieves the state changes within the PX4

        mavros_msgs::State current_state_;                               ///< The current state on the pixhawk

        ros::ServiceClient set_mode_client_;                             ///< Issues the state change commands

        ros::Time last_request_ = ros::Time::now();                      ///< Keeps track of the time since the last
                                                                         ///< request.

        /**
         * Callback function from mavros. Updates the current state.
         *
         * @param msg The new state.
         */
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
 
    public:

        /**
         * Sets ut the mavros state link so it can issue and retrieve state changes within PX4.
         */
        MavrosStateLink();

        /**
         * @return The current state the state subscriber has obtained.
         */
        mavros_msgs::State getCurrentState();

        /**
         * Checks if the current state is the state we want the Pixhawk to be in. If not, this function will issue
         * a state change command at the FSM's predefined refresh rate. 
         *
         * @param mode               The mode to attempt to set.
         * @param completion_handler Called when the state change call is responded, will return with a flag whether
         *                           the state change succeeded or not.
         */
        void attemptToSetState(std::string mode, std::function<void (bool)> completion_handler);
    };
}

#endif
