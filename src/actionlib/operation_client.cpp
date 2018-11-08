//
// Created by simengangstad on 25.10.18.
//

#include "actionlib/operation_client.h"
#include "../../include/operations/operation_identifier.h"
#include "../../include/operations/operation_util.h"

#include <fluid_fsm/MoveGoal.h>
#include <fluid_fsm/TakeOffGoal.h>
#include <fluid_fsm/LandGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

void fluid::OperationClient::requestOperationToTargetPoint(geometry_msgs::Pose target_pose, std::function<void (bool)> callback) {

    // This is not suitable, should've been broken into a generic way, but it's not possible as
    // actionlib's Goal objects doesn't have a superclass...
    switch (operation_identifier_) {
        case OperationIdentifier::init: {

            InitActionClient init_action_client(
                    fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier_), true);
            init_action_client.waitForServer();

            fluid_fsm::InitGoal init_goal;
            init_goal.target_pose = target_pose;
            init_action_client.sendGoal(init_goal);
            bool finished_before_timeout = init_action_client.waitForResult(ros::Duration(timeout_value_));

            if (callback) {
                callback(finished_before_timeout && init_action_client.getState().isDone());
            }

        } break;

        case OperationIdentifier::move: {

            MoveActionClient move_action_client(
                    fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier_), true);
            move_action_client.waitForServer();

            fluid_fsm::MoveGoal move_goal;
            move_goal.target_pose = target_pose;
            move_action_client.sendGoal(move_goal);
            bool finished_before_timeout = move_action_client.waitForResult(ros::Duration(timeout_value_));

            if (callback) {
                callback(finished_before_timeout && move_action_client.getState().isDone());
            }

        } break;

        case OperationIdentifier::land: {

            LandActionClient land_action_client(
                    fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier_), true);
            land_action_client.waitForServer();

            fluid_fsm::LandGoal land_goal;
            land_goal.target_pose = target_pose;
            land_action_client.sendGoal(land_goal);
            bool finished_before_timeout = land_action_client.waitForResult(ros::Duration(timeout_value_));

            if (callback) {
                callback(finished_before_timeout && land_action_client.getState().isDone());
            }

        } break;

        case OperationIdentifier::take_off: {

            TakeOffActionClient take_off_action_client(
                    fluid::OperationUtil::descriptionFromOperationIdentifier(operation_identifier_), true);
            take_off_action_client.waitForServer();

            fluid_fsm::TakeOffGoal take_off_goal;
            take_off_goal.target_pose = target_pose;
            take_off_action_client.sendGoal(take_off_goal);
            bool finished_before_timeout = take_off_action_client.waitForResult(ros::Duration(timeout_value_));

            if (callback) {
                callback(finished_before_timeout && take_off_action_client.getState().isDone());
            }

        } break;
    }
}