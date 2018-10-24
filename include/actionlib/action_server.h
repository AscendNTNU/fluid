//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "../operations/operation_identifier.h"
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/MoveGoal.h>

namespace fluid {

    /** \class ActionServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    template<typename GoalConstPtr, typename Action> class ActionServer {

        typedef actionlib::SimpleActionServer<Action> Server;

    private:
        const OperationIdentifier operation_identifier_; ///< The operation identifier for the given operation of
                                                         ///< this action server

        ros::NodeHandle node_handle_;                    ///< The node handle for this action server

        Server actionlib_action_server_;                 ///< Reference to the actionlib server which this action
                                                         ///< server object encapsulates

        /**
         * Gets a description of the operation identifier.
         *
         * @param identifier The operation identifier to get the description from.
         * @return The description of the operation identifier.
         */
        static std::string descriptionFromOperationIdentifier(fluid::OperationIdentifier identifier) {
            std::string description;

            switch (identifier) {
                case fluid::OperationIdentifier::take_off:
                    description = "take_off";
                    break;

                case fluid::OperationIdentifier::move:
                    description = "move";
                    break;

                case fluid::OperationIdentifier::land:
                    description = "land";
                    break;

                default:
                    description = "unknown";
                    break;
            }

            return description;
        }

    public:

        /** Initializes the action server with an operation identifier.
         *
         * @param operation_identifier The operation identifier for the given action server.
         */
        ActionServer(fluid::OperationIdentifier operation_identifier) :
        operation_identifier_(operation_identifier),
        actionlib_action_server_(node_handle_,
                                 ActionServer::descriptionFromOperationIdentifier(operation_identifier),
                                 boost::bind(&ActionServer::execute, this, _1), false) {
            actionlib_action_server_.start();
        }

        /**
        * Executes the action. This will perform the given operation with the operation identifier.
        *
        * @param goal The goal of the action/operation, this will in most cases be a set point.
        */
        void execute(const GoalConstPtr &goal) {

            // Send final pose in set succeeded
            ROS_INFO_STREAM("Hello world");
            actionlib_action_server_.setSucceeded();
        }
    };
}

#endif //FLUID_FSM_ACTION_SERVER_H
