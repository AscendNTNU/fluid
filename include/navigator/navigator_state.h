//
// Created by simengangstad on 08.01.19.
//

#ifndef FLUID_FSM_NAVIGATOR_STATE_H
#define FLUID_FSM_NAVIGATOR_STATE_H

#include "../core/state.h"
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
         * @param node_handle_p Node handle to interact with ROS topics.
         * @param identifier The identifier of the state.
         */
        // TODO: Topic is temporary
        // TODO: Implement refresh rate here
        NavigatorState(fluid::OperationIdentifier identifier) :
        State(std::move(identifier), 
              "state_estimator_pose",
              std::make_shared<fluid::NavigatorPosePublisher>("navigator_pose_topic", 1000), 
              20) {}
    };
}

#endif //FLUID_FSM_NAVIGATOR_STATE_H
