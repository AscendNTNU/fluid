//
// Created by simengangstad on 08.11.18.
//

#include "../../include/actionlib/operation_server.h"
#include "../../include/operations/operation_defines.h"
#include "operations/init_operation.h"
#include "operations/move_operation.h"
#include "operations/land_operation.h"
#include "operations/take_off_operation.h"
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <fluid_fsm/OperationGoal.h>


void fluid::OperationServer::goalCallback() {
    auto goal = actionlib_action_server_.acceptNewGoal();
    geometry_msgs::Pose target_pose = goal->target_pose;
    std_msgs::String operation_identifier = goal->type;

    mavros_msgs::PositionTarget position_target;
    position_target.position = target_pose.position;

    if (operation_identifier.data == fluid::operation_identifiers::INIT) {
        position_target.position.x = 0.0;
        position_target.position.y = 0.0;
        position_target.position.z = 0.0;
        next_operation_p_ = std::make_shared<fluid::InitOperation>(position_target, refresh_rate_);

    }
    else if (operation_identifier.data == fluid::operation_identifiers::TAKE_OFF) {
        next_operation_p_ = std::make_shared<fluid::TakeOffOperation>(position_target, refresh_rate_);

    }
    else if (operation_identifier.data == fluid::operation_identifiers::MOVE) {
        next_operation_p_ = std::make_shared<fluid::MoveOperation>(position_target, refresh_rate_);

    }
    else if (operation_identifier.data == fluid::operation_identifiers::LAND) {
        next_operation_p_ = std::make_shared<fluid::LandOperation>(position_target, refresh_rate_);
    }

    new_operation_requested_ = true;

    ROS_INFO_STREAM("New operation requested: " << operation_identifier.data.c_str());
}

void fluid::OperationServer::preemptCallback() {
    ROS_INFO("%s: Preempted", current_operation_p_->identifier.c_str());
    actionlib_action_server_.setPreempted();
}

void fluid::OperationServer::start() {

    ROS_INFO("Operation server running and listening.");

    ros::Rate rate(refresh_rate_);

    while (ros::ok()) {

        if (new_operation_requested_) {
            current_operation_p_ = next_operation_p_;
            next_operation_p_.reset();
            new_operation_requested_ = false;
        }

        // We have an operation to execute.
        if (current_operation_p_) {
            current_operation_p_->perform(
                    [&]() -> bool {
                        // We abort current mission if there is a new operation.
                        if (new_operation_requested_) {
                            ROS_INFO_STREAM("Aborting current operation: " << 
                                            current_operation_p_->identifier.c_str());
                        }

                        return new_operation_requested_;
                    },

                    [&](bool completed) {
                        last_state_p_ = current_operation_p_->getFinalStatePtr();

                        if (completed) {
                            actionlib_action_server_.setSucceeded();
                        }
                        else {
                            actionlib_action_server_.setAborted();
                        }
                    });

            current_operation_p_.reset();
            //ROS_INFO_STREAM("Operation finished, state is now: " << last_state_p_->identifier << " position target: " << last_state_p_->position_target);
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {
            if (last_state_p_) {
                last_state_p_->perform([&]() -> bool {
                    // We abort the execution of the current state if there is a new operation.
                    return new_operation_requested_;
                });
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}