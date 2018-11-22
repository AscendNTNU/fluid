//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_H
#define FLUID_FSM_MAVROS_STATE_H

#include "../core/state.h"
#include "../core/operation/operation.h"
#include "mavros_pose_publisher.h"

#include <memory>
#include <utility>

#include <ros/ros.h>

namespace fluid {
    /**
     * \class MavrosState
     * \brief Encapsulates a state which publishes poses through mavros.
     */
    class MavrosState: public State {

    private:


        ros::Subscriber pose_subscriber_;                        ///< Retrieves poses from mavros

    protected:

        ros::NodeHandlePtr node_handle_p;                        ///< Node handle for the mavros pose publisher

        geometry_msgs::PoseStamped current_position_;            ///< Keeps track of where the drone is during this
                                                                 ///< state in terms of mavros.

        /**
         * Gets fired when mavros publishes a pose on the topic "mavros/local_position/pose".
         */
        void poseCallback(geometry_msgs::PoseStampedConstPtr pose);

    public:

        /**
         * Initiializes the mavros state with an identifier.
         *
         * @param identifier The identifier of the state.
         * @param node_handle_p Node handle to interact with ROS topics.
         */
        MavrosState(ros::NodeHandlePtr node_handle_p, fluid::OperationIdentifier identifier) :
        State(std::move(identifier), std::make_shared<fluid::MavrosPosePublisher>(node_handle_p, 1000), 20),
        node_handle_p(node_handle_p),
        pose_subscriber_(node_handle_p->subscribe("mavros/local_position/pose", 1000, &MavrosState::poseCallback, this))
        {}
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
