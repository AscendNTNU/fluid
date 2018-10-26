//
// Created by simengangstad on 26.10.18.
//

#ifndef FLUID_FSM_POSE_PUBLISHER_H
#define FLUID_FSM_POSE_PUBLISHER_H

#include <geometry_msgs/PoseStamped.h>

namespace fluid {
    /**
     * \class Publisher
     * \brief Defines an interface for publishing pose. States implement this interface so one can swap out the method
     *        for publishing.
     */
    class PosePublisher {
        /**
         * Publishes a pose.
         *
         * @param pose_stamped The pose that ought to be published.
         */
        virtual void publish(geometry_msgs::PoseStamped pose_stamped) = 0;
    };
}

#endif //FLUID_FSM_POSE_PUBLISHER_H
