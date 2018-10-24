//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_ACTION_CLIENT_H
#define FLUID_FSM_ACTION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../operations/operation_identifier.h"
#include "../operations/operation_util.h"
#include "../core/state.h"
#include <fluid_fsm/MoveGoal.h>
#include <fluid_fsm/TakeOffGoal.h>
#include <fluid_fsm/LandGoal.h>
#include <geometry_msgs/Pose.h>

namespace fluid {

    /** \class ActionClient
     *  \brief Encapsulates a ROS action client which requests operations.
     */
    template<typename Action> class ActionClient {

        typedef actionlib::SimpleActionClient<Action> Client;

    private:

        const OperationIdentifier operation_identifier_; ///< The identifier of the operation, and thus the action server this
                                                         ///< action client will send requests to

        const unsigned int timeout_value_;               ///< The time the the client will wait for a response from
                                                         ///< the server

        Client actionlib_client_;                        ///< Reference to the action lib client which this action
                                                         ///< client class encapsulates

    public:

        /** Initializes the action client with an operation identifier.
         *
         * @param operation_identifier Specifies the kind of operation request.
         */
        ActionClient(fluid::OperationIdentifier operation_identifier, unsigned int timeout_value) : operation_identifier_(operation_identifier), timeout_value_(timeout_value), actionlib_client_(fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier), true) {
            actionlib_client_.waitForServer();
        }

        /**
         * Requests an operation with a given target pose. This function will send a request to a server
         * listening on the "operation identifier domain".
         *
         * @param target_pose The target pose of the operation.
         * @param callback Gets fired when the operation finished, includes a flag whether the operation finished
         *                 before timeout or not.
         */
        void requestOperationToTargetPoint(geometry_msgs::Pose target_pose, std::function<void (bool)> callback) {

            // TODO: This fails at compile time because action lib client refers to only one of the the three types...

            switch (operation_identifier_) {
                case OperationIdentifier::move: {

                    fluid_fsm::MoveGoal move_goal;
                    move_goal.target_pose = target_pose;
                    actionlib_client_.sendGoal(move_goal);
                    break;
                }

                case OperationIdentifier::land: {

                    fluid_fsm::LandGoal land_goal;
                    land_goal.target_pose = target_pose;
                    actionlib_client_.sendGoal(land_goal);
                    break;
                }

                case OperationIdentifier::take_off: {

                    fluid_fsm::TakeOffGoal take_off_goal;
                    take_off_goal.target_pose = target_pose;
                    actionlib_client_.sendGoal(take_off_goal);
                    break;
                }

                default: {

                    callback(false);
                    return;
                }
            }

            bool finished_before_timeout = actionlib_client_.waitForResult(ros::Duration(timeout_value_));

            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = actionlib_client_.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());

                if (callback) {
                    callback(true);
                }
            }
            else {
                if (callback) {
                    callback(false);
                }
            }
        }
    };
}


#endif //FLUID_FSM_ACTION_CLIENT_H
