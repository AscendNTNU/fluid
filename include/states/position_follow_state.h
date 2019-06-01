//
// Created by simengangstad on 07.03.19.
//

#ifndef FLUID_FSM_POSITION_FOLLOW_STATE_H
#define FLUID_FSM_POSITION_FOLLOW_STATE_H

#include "../../include/core/core.h"
#include "../core/state.h"
#include "state_identifier.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace fluid {

    /** \class PositionFollow
     *  \brief Represents the state where the drone is following a position.
     */
    class PositionFollowState: public State {

    private: 

        bool has_target_ = false;                                               ///< Indicates whether a valid target 
                                                                                ///< was pubilshed

        bool set_standby_position_ = false;                                     ///< Determines whether we set a target
                                                                                ///< pose when we don't have an object 
                                                                                ///< to track.

        geometry_msgs::Pose object_target_pose_;                                ///< The pose to follow.

        mavros_msgs::PositionTarget calculated_pose_;                           ///< The pose we want the drone to be
                                                                                ///< at so it's on some distance from 
                                                                                ///< the object target pose.

        ros::Subscriber object_target_pose_subscriber_;                         ///< Retrieves the pose of the object 
                                                                                ///< we're to follow

        void objectTargetPoseCallback(geometry_msgs::Pose object_target_pose);  ///< Gets called when a target pose is 
                                                                                ///< published

    public:

        /** Initializes the move state.
         */
        explicit PositionFollowState() : 
        State(fluid::StateIdentifier::PositionFollow, fluid::PX4::Offboard, true), 
        object_target_pose_subscriber_(node_handle_.subscribe("perception/target", 
                                                              Core::message_queue_size, 
                                                              &PositionFollowState::objectTargetPoseCallback, 
                                                              this)) {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;
    };
}

#endif
