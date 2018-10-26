//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "../operations/operation_identifier.h"
#include "../operations/operation_util.h"

namespace fluid {

    /** \class OperationServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    template<typename Action, typename GoalConstPtr> class OperationServer {

        typedef actionlib::SimpleActionServer<Action> Server;

    private:

        ros::NodeHandle node_handle_;                    ///< The node handle for this operation server

        Server actionlib_action_server_;                 ///< Reference to the ROS action server which this operation
                                                         ///< server class encapsulates

    public:

        /** Initializes the operation server with an operation identifier.
         *
         * @param operation_identifier The operation identifier, which is essentially the domain this server provides
         *                             a service on.
         */
        OperationServer(fluid::OperationIdentifier operation_identifier) :
        actionlib_action_server_(node_handle_,
                                 fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier),
                                 boost::bind(&OperationServer::execute, this, _1), false) {
            actionlib_action_server_.start();
        }

        /**
        * Executes the operation. This will perform the given operation with the operation identifier.
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
