//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_OPERATION_CLIENT_H
#define FLUID_FSM_OPERATION_CLIENT_H

#include "operations/operation_identifier.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/TakeOffAction.h>
#include <fluid_fsm/LandAction.h>


namespace fluid {

    /** \class OperationClient
     *  \brief Encapsulates a ROS action client which requests operations.
     */
    class OperationClient {

    private:

        typedef actionlib::SimpleActionClient<fluid_fsm::MoveAction> MoveActionClient;
        typedef actionlib::SimpleActionClient<fluid_fsm::LandAction> LandActionClient;
        typedef actionlib::SimpleActionClient<fluid_fsm::TakeOffAction> TakeOffActionClient;

        const OperationIdentifier operation_identifier_; ///< The identifier of the operation, and thus the operation
                                                         ///< server this operation client will send requests to

        const unsigned int timeout_value_;               ///< The time the the client will wait for a response from
                                                         ///< the server

    public:

        /** Initializes the operation client with an operation identifier and a timeout value.
         *
         * @param operation_identifier Specifies the kind of operation request.
         * @param timeout_value The time the operation client waits for a response from the action server.
         */
        OperationClient(fluid::OperationIdentifier operation_identifier, unsigned int timeout_value) :
        operation_identifier_(operation_identifier),
        timeout_value_(timeout_value) {}

        /**
         * Requests an operation with a given target pose. This function will send a request to a server
         * listening on the "operation identifier domain".
         *
         * @param target_pose The target pose of the operation.
         * @param completion_handler Gets fired when the operation finished, includes a flag whether the operation finished
         *                           before timeout or not.
         */
        void requestOperationToTargetPoint(geometry_msgs::Pose target_pose, std::function<void (bool)> completion_handler);
    };
}

#endif //FLUID_FSM_OPERATION_CLIENT_H
