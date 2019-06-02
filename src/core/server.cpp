//
// Created by simengangstad on 08.11.18.
//

#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <fluid/OperationGoal.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <assert.h>
#include <algorithm>

#include "server.h"
#include "operation_identifier.h"
#include "init_operation.h"
#include "move_operation.h"
#include "land_operation.h"
#include "take_off_operation.h"
#include "move_oriented_operation.h"
#include "position_follow_operation.h" 
#include "core.h"

fluid::Server::Server() : actionlib_server_(node_handle_, 
                                                   "fluid_fsm_operation",
                                                   false) {
    
    actionlib_server_.registerGoalCallback(boost::bind(&Server::goalCallback, this));
    actionlib_server_.registerPreemptCallback(boost::bind(&Server::preemptCallback, this));

    actionlib_server_.start();
}


void fluid::Server::goalCallback() {

    // We accept the new goal and initialize variables for target pose and the type of operation identifier.
    // This is necessary in order to modify some of them before we initiate the different operations further down.
    // E.g. the init operation shouldn't be called with a different pose than (0, 0, 0), so we make sure this is the
    // case.
    
    if (actionlib_server_.isActive()) {
        actionlib_server_.setAborted();
    }

    auto goal = actionlib_server_.acceptNewGoal();
    geometry_msgs::Pose target_pose = goal->target_pose;
    std_msgs::String operation_identifier = goal->type;

    ROS_INFO_STREAM("New operation requested: " << operation_identifier);

    // Check first if a boundry is defined (!= 0). If there is a boundry the position target is clamped to 
    // min and max.
    if (fluid::Core::minX != 0 || fluid::Core::maxX != 0) {
        target_pose.position.x = std::max(fluid::Core::minX, std::min(target_pose.position.x, fluid::Core::maxX));
    }

    if (fluid::Core::minY != 0 || fluid::Core::maxY != 0) { 
        target_pose.position.y = std::max(fluid::Core::minY, std::min(target_pose.position.y, fluid::Core::maxY));
    }

    if (fluid::Core::minZ != 0 || fluid::Core::maxZ != 0) {
        target_pose.position.z = std::max(fluid::Core::minZ, std::min(target_pose.position.z, fluid::Core::maxZ));
    }

    // Update the status with the target pose
    Core::getStatusPublisherPtr()->status.target_pose_x = target_pose.position.x;
    Core::getStatusPublisherPtr()->status.target_pose_y = target_pose.position.y;
    Core::getStatusPublisherPtr()->status.target_pose_z = target_pose.position.z;
    
    // The goal/request is sent with a geometry_pose, so we have to convert to position_target for mavros.
    mavros_msgs::PositionTarget position_target;
    position_target.position = target_pose.position;

    tf2::Quaternion quat(target_pose.orientation.x, 
                         target_pose.orientation.y, 
                         target_pose.orientation.z, 
                         target_pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
    // it to zero. 
    position_target.yaw = std::isnan(yaw) ? 0.0 : yaw;


    std::map<std::string, std::string> operation_state_identifier_map = {
        {fluid::OperationIdentifier::Init, fluid::StateIdentifier::Init},
        {fluid::OperationIdentifier::TakeOff, fluid::StateIdentifier::TakeOff},
        {fluid::OperationIdentifier::Move, fluid::StateIdentifier::Move},
        {fluid::OperationIdentifier::Land, fluid::StateIdentifier::Land},
        {fluid::OperationIdentifier::PositionFollow, fluid::StateIdentifier::PositionFollow},
    };

    // Point the next operation pointer to the newly initialized operation.
/*    if (operation_identifier.data == fluid::OperationIdentifier::Init) {
        position_target.position.x = 0.0;
        position_target.position.y = 0.0;
        position_target.position.z = 0.0;
        next_operation_p_ = std::make_shared<fluid::InitOperation>(position_target);

    }
    else if (operation_identifier.data == fluid::OperationIdentifier::TakeOff) {
        next_operation_p_ = std::make_shared<fluid::TakeOffOperation>(position_target);

    }
    else if (operation_identifier.data == fluid::OperationIdentifier::Move) {
        next_operation_p_ = std::make_shared<fluid::MoveOperation>(position_target);
    }
    else if (operation_identifier.data == fluid::OperationIdentifier::Land) {
        next_operation_p_ = std::make_shared<fluid::LandOperation>(position_target);
    }
    else if (operation_identifier.data == fluid::OperationIdentifier::PositionFollow) {
        next_operation_p_ = std::make_shared<fluid::PositionFollowOperation>();
    }
    else if (operation_identifier.data == fluid::OperationIdentifier::MoveOriented) {
        next_operation_p_ = std::make_shared<fluid::MoveOrientedOperation>(position_target);
    }*/

    next_operation_p_ = std::make_shared<fluid::Operation>(operation_identifier, operation_state_identifier_map[operation_identifier.data], "", position_target);

    new_operation_requested_ = true;
}

void fluid::Server::preemptCallback() {
    actionlib_server_.setPreempted();
}

void fluid::Server::start() {

    ros::Rate rate(fluid::Core::refresh_rate);

    // Main loop of Fluid FSM. This is where all the magic happens. If a new operaiton is requested, the 
    // new operation requested flag is set and we set up the requirements for that operation to run. When it
    // it runs we check every tick if a new operation is requested and abort from the current operation if
    // that is the case. 
    while (ros::ok()) {

        // Setup for the new operation.
        if (new_operation_requested_) {
            current_operation_p_ = next_operation_p_;
            next_operation_p_.reset();
            new_operation_requested_ = false;
        }

        // Execute the operation if there is any
        if (current_operation_p_) {

            fluid::Core::getStatusPublisherPtr()->status.current_operation = current_operation_p_->identifier;

            current_operation_p_->perform(

                [&]() -> bool { return new_operation_requested_; },

                [&](bool completed) {
                    // We completed the operation and want to end at the final state of the operation (e.g. hold)
                    // state for move. One can think of this step as making sure that the state machine is at a
                    // state where it's easy to execute a new operation.
                    last_state_p_ = current_operation_p_->getFinalStatePtr();


                    // Will notify the operation client what the outcome of the operation was. This will end up
                    // calling the callback that the operation client set up for completion.
                    
                    if (!actionlib_server_.isActive()) {
                        return;
                    }

                    if (completed) {
                        ROS_INFO_STREAM("Operation completed.");
                        actionlib_server_.setSucceeded();
                    }
                    else {
                        actionlib_server_.setAborted();
                    }
                });

            current_operation_p_.reset();
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {

            fluid::Core::getStatusPublisherPtr()->status.current_operation = "none";

            if (last_state_p_) {

                fluid::Core::getStatusPublisherPtr()->status.current_state = last_state_p_->identifier;
                last_state_p_->perform([&]() -> bool {
                    // We abort the execution of the current state if there is a new operation.
                    return new_operation_requested_;
                });
            }
        }

        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();
    }
}