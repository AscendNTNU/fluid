//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <actionlib/server/simple_action_server.h>
#include "../operations/operation_identifier.h"

namespace fluid {

    /** \class ActionServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    template <typename GoalConstPtr, typename Action>
    class ActionServer {
    private:
        const OperationIdentifier operation_identifier_; ///< The operation identifier for the given operation of
                                                         ///< this action server

        /**
         * Executes the action. This will perform the given operation with the operation identifier.
         *
         * @param goal The goal of the action/operation, this will in most cases be a set point.
         * @param action_server The action server which performed the operation.
         */
        void execute(const GoalConstPtr& goal, actionlib::SimpleActionServer<Action>* action_server_p);

    public:
        /** Initializes the action server with an operation identifier.
         *
         * @param operation_identifier The operation identifier for the given action server.
         */
        ActionServer(OperationIdentifier operation_identifier);
    };
}

#endif //FLUID_FSM_ACTION_SERVER_H
