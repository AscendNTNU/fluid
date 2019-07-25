//
// Created by simengangstad on 09.06.19.
//

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "rotate_state.h"
#include "pose_util.h"
#include "core.h"

bool fluid::RotateState::hasFinishedExecution() {
    bool atPositionTarget = PoseUtil::distanceBetween(current_pose_, setpoint) < fluid::Core::distance_completion_threshold && 
    	   				 	std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;



    bool atYawTarget = std::abs(PoseUtil::angleBetween(current_pose_.pose.orientation, setpoint.yaw)) < fluid::Core::yaw_completion_threshold; 

    return atYawTarget && atPositionTarget;
}

void fluid::RotateState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z;
}

void fluid::RotateState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;
}
