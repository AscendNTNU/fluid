//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <ros/ros.h>
#include <fluid_fsm/OperationAction.h>
#include <fluid_fsm/OperationGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "core/operation/operation.h"

namespace fluid {

    /** \class OperationServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    class OperationServer {

        typedef actionlib::SimpleActionServer<fluid_fsm::OperationAction> Server;

    private:

        ros::NodeHandle node_handle_;                                ///< The node handle for the operation server

        Server actionlib_action_server_;                             ///< Reference to the ROS action server which the
                                                                     ///< operation server class encapsulates
    public:

        std::function<void (std::shared_ptr<fluid::Operation>)> operationRequestedCallback;  ///< Callback for the
                                                                                             ///< operation requested

        /**
         * Initializes the operation server.
         */
        OperationServer() : actionlib_action_server_(node_handle_,
                                                     "fluid_fsm_operation",
                                                     boost::bind(&OperationServer::execute, this, _1), false) {
            actionlib_action_server_.start();
        }

       /**
        * Executes the operation. This will perform the given operation with the operation identifier.
        *
        * @param goal The goal of the action/operation, this will in most cases be a set point.
        */
        void execute(const fluid_fsm::OperationGoalConstPtr &goal);
    };
}

#endif //FLUID_FSM_ACTION_SERVER_H
