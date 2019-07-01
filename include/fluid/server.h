//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_SERVER_H
#define FLUID_FSM_SERVER_H

#include <ros/ros.h>
#include <fluid/OperationAction.h>
#include <actionlib/server/simple_action_server.h>

#include "operation.h"

namespace fluid {

    /** \class Server
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    class Server {

        typedef actionlib::SimpleActionServer<fluid::OperationAction> ActionlibServer;

    private:

        ros::NodeHandle node_handle_;                                ///< Used to instantiate the ROS action server

        ActionlibServer actionlib_server_;                           ///< Reference to the ROS action server which the
                                                                     ///< server class encapsulates

        std::shared_ptr<fluid::Operation> current_operation_p_;      ///< The current operation executing.

        std::shared_ptr<fluid::Operation> next_operation_p_;         ///< Next operation requested.

        std::shared_ptr<fluid::State> last_state_p_;                 ///< Pointer to the last state executed.

        bool new_operation_requested_ = false;                       ///< Determines whether a new operation was
                                                                     ///< requested.

    public:

        /**
         * Initializes the server.
         */
        Server();

        /**
         * Gets fired when the server receives a new goal.
         */
        void goalCallback();

        /**
         * Gets fired when the operation gets preempted.
         */
        void preemptCallback();

       /**
        * Starts the server and awaits requests.
        */
        void start();
    };
}

#endif
