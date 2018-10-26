//
// Created by simengangstad on 26.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_SUBSCRIBER_H
#define FLUID_FSM_MAVROS_STATE_SUBSCRIBER_H

#include <ros/ros.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


namespace fluid {
    /**
     * \class MavrosStateSubscriber
     * \brief Subscribes to state changes within the PixHawk.
     */
    class MavrosStateSubscriber {

    private:

        ros::Subscriber state_subscriber;                               ///< ROS subscriber which subscribes to
                                                                        ///< state changes within the pixhawk

        /**
         * Callback function from mavros. Updates the current state.
         *
         * @param msg The new state.
         */
        void state_callback(const mavros_msgs::State::ConstPtr& msg);

    public:

        mavros_msgs::State current_state;                               ///< The current state on the pixhawk

        /**
         * Sets up the mavros state subscriber so it can listen to state changes on the pixhawk.
         *
         * @param node_handle The ros node handle to listen from.
         * @param message_queue_size The size of the message buffer.
         */
        MavrosStateSubscriber(ros::NodeHandle node_handle, unsigned int message_queue_size) :
        state_subscriber(node_handle.subscribe<mavros_msgs::State>("mavros/state",
                                                                   message_queue_size,
                                                                   &MavrosStateSubscriber::state_callback,
                                                                   this)) {}
    };
}

#endif //FLUID_FSM_MAVROS_STATE_SUBSCRIBER_H
