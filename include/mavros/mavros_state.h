//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_H
#define FLUID_FSM_MAVROS_STATE_H

#include "../core/state.h"
#include "mavros_pose_publisher.h"

#include <memory>

#include <ros/ros.h>

namespace fluid {
    /**
     * \class MavrosState
     * \brief Encapsulates a state which publishes poses through mavros.
     */
    class MavrosState: public State {

    private:
        ros::NodeHandle node_handle_;                            ///< Node handle for the mavros pose publisher

    public:

        /**
         * Initiializes the mavros state with an identifier.
         *
         * @param identifier The identifier of the state.
         */
        MavrosState(std::string identifier) :
        State(identifier, std::make_shared<fluid::MavrosPosePublisher>(node_handle_, 1000), 20) {}
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
