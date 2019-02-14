//
// Created by simengangstad on 08.01.19.
//

#ifndef FLUID_FSM_NAVIGATOR_POSE_PUBLISHER_H
#define FLUID_FSM_NAVIGATOR_POSE_PUBLISHER_H

#include "../core/pose_publisher.h"
#include <ros/ros.h>
#include <string>
#include <mavros_msgs/PositionTarget.h>

namespace fluid {

    /**
     * \class NavigatorPosePublisher
     * \brief Publishes poses on the specified topic for the navigator.
     */
    class NavigatorPosePublisher: public PosePublisher {
    private:

        ros::NodeHandle node_handle_;                           ///< Used to set up the publisher.

        ros::Publisher local_position_publisher_;               ///< Ros publisher which publishes the poses

    public:

        /**
         * Sets up the navigator pose publisher so it can publish poses.
         *
         * @param node_handle_p Node handle to publish from.
         */
        NavigatorPosePublisher(std::string topic);

        /**
         * Method overridden from interface.
         */
        void publish(mavros_msgs::PositionTarget position_target) override;
    };
}

#endif //FLUID_FSM_NAVIGATOR_POSE_PUBLISHER_H
