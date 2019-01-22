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

    public:

        /**
         * Initiializes the mavros state with an identifier.
         *
         * @param node_handle_p Node handle to interact with ROS topics.
         * @param identifier The identifier of the state.
         */
        // TODO: Unify refresh rate
        MavrosState(ros::NodeHandlePtr node_handle_p, 
                    fluid::OperationIdentifier identifier,
                    unsigned int refresh_rate) :
        State(node_handle_p,
              std::move(identifier),
              "mavros/local_position/pose", 
              std::make_shared<fluid::MavrosPosePublisher>(node_handle_p, 1000), 
              refresh_rate) {}
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
