//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <fluid_fsm/OperationAction.h>
#include <fluid_fsm/OperationGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "core/operation/operation.h"
#include "../../include/core/status_publisher.h"

namespace fluid {

    /** \class OperationServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    class OperationServer {

        typedef actionlib::SimpleActionServer<fluid_fsm::OperationAction> Server;

    private:

        ros::NodeHandle node_handle_;                                ///< Used to instantiate the ROS action server

        Server actionlib_action_server_;                             ///< Reference to the ROS action server which the
                                                                     ///< operation server class encapsulates

        std::shared_ptr<fluid::Operation> current_operation_p_;      ///< The current operation executing.

        std::shared_ptr<fluid::Operation> next_operation_p_;         ///< Next operation requested.

        std::shared_ptr<fluid::State> last_state_p_;                 ///< Pointer to the last state executed.

        bool new_operation_requested_ = false;                       ///< Determines whether a new operation was
                                                                     ///< requested.

    public:

        /**
         * Initializes the operation server.
         */
        OperationServer();

        /**
         * Gets fired when the operation server receives a new goal.
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

#endif //FLUID_FSM_ACTION_SERVER_H
