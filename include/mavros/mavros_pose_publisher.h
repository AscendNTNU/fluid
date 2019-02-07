//
// Created by simengangstad on 26.10.18.
//

#ifndef FLUID_FSM_MAVROS_POSE_PUBLISHER_H
#define FLUID_FSM_MAVROS_POSE_PUBLISHER_H

#include "../core/pose_publisher.h"
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {

    /**
     * \class MavrosPosePublisher
     * \brief Publishes poses on the mavros/local/setpoint_position/local topic
     */
    class MavrosPosePublisher: public PosePublisher {
    private:

        ros::NodeHandle node_handle_;                           ///< Used to set up the publisher.

        ros::Publisher local_position_publisher_;               ///< Ros publisher which publishes the poses

    public:

        /**
         * Sets up the mavros pose publisher so it can publish poses.
         *
         * @param message_queue_size The size of the message buffer.
         */
        MavrosPosePublisher(unsigned int message_queue_size) :
        local_position_publisher_(node_handle_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",
                                                                                         message_queue_size)) {}

        /**
         * Method overridden from interface.
         */
        void publish(mavros_msgs::PositionTarget position_target) override;
    };
}

#endif //FLUID_FSM_MAVROS_POSE_PUBLISHER_H
