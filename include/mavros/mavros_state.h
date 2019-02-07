//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_H
#define FLUID_FSM_MAVROS_STATE_H

#include "../core/state.h"
#include "../core/core.h"
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
         * @param identifier The identifier of the state.
         */
        MavrosState(fluid::StateIdentifier identifier);
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
