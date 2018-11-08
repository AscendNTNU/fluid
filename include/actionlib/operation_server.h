//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_ACTION_SERVER_H
#define FLUID_FSM_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "operations/operation_identifier.h"
#include "operations/operation_util.h"
#include "core/operation/operation.h"
#include "operations/init_operation.h"
#include "operations/move_operation.h"
#include "operations/land_operation.h"
#include "operations/take_off_operation.h"
#include <mavros_msgs/PositionTarget.h>

namespace fluid {

    /** \class OperationServer
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    template<typename Action, typename GoalConstPtr> class OperationServer {

        typedef actionlib::SimpleActionServer<Action> Server;

    private:

        const fluid::OperationIdentifier operation_identifier_;      ///< Specifies the kind of operation the
                                                                     ///< operation server responds to

        ros::NodeHandle node_handle_;                                ///< The node handle for the operation server

        Server actionlib_action_server_;                             ///< Reference to the ROS action server which the
                                                                     ///< operation server class encapsulates
    public:

        std::function<void (std::shared_ptr<fluid::Operation>)> operationRequestedCallback;  ///< Callback for the
                                                                                             ///< operation requested

        /** Initializes the operation server with an operation identifier.
         *
         * @param operation_identifier The operation identifier, which is essentially the domain this server provides
         *                             a service on.
         */
        OperationServer(fluid::OperationIdentifier operation_identifier) :
        operation_identifier_(operation_identifier),
        actionlib_action_server_(node_handle_,
                                 fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier_),
                                 boost::bind(&OperationServer::execute, this, _1), false) {
            actionlib_action_server_.start();
        }

       /**
        * Executes the operation. This will perform the given operation with the operation identifier.
        *
        * @param goal The goal of the action/operation, this will in most cases be a set point.
        */
        void execute(const GoalConstPtr &goal) {

           mavros_msgs::PositionTarget position_target;
           position_target.position = goal->target_pose.position;

           ROS_INFO_STREAM("Got operation request for " << operation_identifier_);

           switch (operation_identifier_) {
                case fluid::OperationIdentifier::init: {
                    std::shared_ptr<fluid::InitOperation> init_operation = std::make_shared<fluid::InitOperation>(position_target);
                    operationRequestedCallback(init_operation);
                    break;
                }

                case fluid::OperationIdentifier::move: {
                    std::shared_ptr<fluid::MoveOperation> move_operation = std::make_shared<fluid::MoveOperation>(position_target);
                    operationRequestedCallback(move_operation);
                    break;
                }

                case fluid::OperationIdentifier::land: {
                    std::shared_ptr<fluid::LandOperation> land_operation = std::make_shared<fluid::LandOperation>(position_target);
                    operationRequestedCallback(land_operation);
                    break;
                }

                case fluid::OperationIdentifier::take_off: {
                    std::shared_ptr<fluid::TakeOffOperation> take_off_operation = std::make_shared<fluid::TakeOffOperation>(position_target);
                    operationRequestedCallback(take_off_operation);
                    break;
                }
            }

           actionlib_action_server_.setPreempted();
       }

        /**
         * Notifies the action lib client that the server finished executing.
         */
        void finishedExecuting() {
            // TODO: Some result here?
            actionlib_action_server_.setSucceeded();
        }

        /**
         * Notifies the action lib client that the server aborted the current operation.
         */
        void abort() {
            // TODO: Some result here?
            //actionlib_action_server_.setAborted();
        }
    };
}

#endif //FLUID_FSM_ACTION_SERVER_H
