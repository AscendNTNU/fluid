#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "rotate_state.h"
#include "util.h"
#include "core.h"

bool fluid::RotateState::hasFinishedExecution() const {
    bool atPositionTarget = Util::distanceBetween(getCurrentPose().pose.position, initial_position) < fluid::Core::distance_completion_threshold && 
    	   				 	std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;



    // bool atYawTarget = std::abs(Util::angleBetween(current_pose_.pose.orientation, setpoint.yaw)) < fluid::Core::yaw_completion_threshold; 

    return /*atYawTarget &&*/ atPositionTarget;
}

void fluid::RotateState::initialize() {
    initial_position.x = getCurrentPose().pose.position.x;
    initial_position.y = getCurrentPose().pose.position.y;
    initial_position.z = getCurrentPose().pose.position.z;
}

std::vector<std::vector<double>> fluid::RotateState::getSplineForPath(const std::vector<geometry_msgs::Point>& path) {
    return Util::getSplineForSetpoint(initial_position, initial_position);
}

fluid::ControllerType fluid::RotateState::getPreferredController() const {
    return ControllerType::Positional; 
}
