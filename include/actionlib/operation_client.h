#include <utility>

//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_OPERATION_CLIENT_H
#define FLUID_FSM_OPERATION_CLIENT_H

#include "../core/operation/operation.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/TakeOffAction.h>
#include <fluid_fsm/InitAction.h>
#include <fluid_fsm/LandAction.h>
#include <core/operation/operation.h>


namespace fluid {

    /** \class OperationClient
     *  \brief Encapsulates a ROS action client which requests operations.
     */
    class OperationClient {

    private:

        const unsigned int timeout_value_;               ///< The time the the client will wait for a response from
                                                         ///< the server

    public:

        /** Initializes the operation client with a timeout value.
         *
         * @param timeout_value The time the operation client waits for a response from the operation server.
         */
        OperationClient(unsigned int timeout_value) : timeout_value_(timeout_value) {}

        /**
         * Requests an operation with a given target pose. This function will send a request to a server
         * listening on the fluid operation domain.
         *
         * @param operation_identifier The type of operation to execute.
         * @param target_pose The target pose of the operation.
         * @param completion_handler Gets fired when the operation finished, includes a flag whether the operation finished
         *                           before timeout or not.
         */
        void requestOperation(fluid::OperationIdentifier operation_identifier,
                              geometry_msgs::Pose target_pose,
                              std::function<void (bool)> completion_handler);
    };
}

#endif //FLUID_FSM_OPERATION_CLIENT_H
