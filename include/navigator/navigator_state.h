//
// Created by simengangstad on 08.01.19.
//

#ifndef FLUID_FSM_NAVIGATOR_STATE_H
#define FLUID_FSM_NAVIGATOR_STATE_H

#include "../core/state.h"
#include "../core/core.h"
#include "../core/operation/operation.h"
#include "navigator_pose_publisher.h"

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {
    /**
     * \class NavigatorState
     * \brief Encapsulates a state which publishes poses through the navigator interface.
     */
    class NavigatorState: public State {

    public:

        /**
         * Initiializes the navigator state with an identifier.
         *
         * @param identifier The identifier of the state.
         */
        // TODO: Topic is temporary
        NavigatorState(fluid::OperationIdentifier identifier);
    };
}

#endif //FLUID_FSM_NAVIGATOR_STATE_H
