//
// Created by simengangstad on 11.10.18.
//

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "move_state.h"
#include "pose_util.h"
#include "core.h"

bool fluid::MoveState::hasFinishedExecution() {
    bool atPositionTarget = PoseUtil::distanceBetween(current_pose_, setpoint) < fluid::Core::distance_completion_threshold && 
    	   				 	std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;

    bool atYawTarget = std::abs(PoseUtil::angleBetween(current_pose_.pose.orientation, setpoint.yaw)) < fluid::Core::yaw_completion_threshold; 

    return atYawTarget && atPositionTarget;
}

void fluid::MoveState::initialize() {
    if (setpoint.position.z <= 0.1) {
		setpoint.position.z = fluid::Core::default_height;
	}
}

void fluid::MoveState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}