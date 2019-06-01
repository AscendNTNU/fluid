#include "../../include/states/position_follow_state.h"
#include "../../include/core/type_mask.h"


#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void fluid::PositionFollowState::objectTargetPoseCallback(geometry_msgs::Pose object_target_pose) {
	
    if (object_target_pose_.position.x == object_target_pose.position.x &&
        object_target_pose_.position.y == object_target_pose.position.y &&
        object_target_pose_.position.z == object_target_pose.position.z) {

        // Same target, so we don't do any new calculations.
        return;
    }

    Core::getStatusPublisherPtr()->status.target_pose_x = object_target_pose.position.x;
    Core::getStatusPublisherPtr()->status.target_pose_y = object_target_pose.position.y;
    Core::getStatusPublisherPtr()->status.target_pose_z = fluid::Core::positionFollowHeight;
 
    set_standby_position_ = false;
    has_target_ = true;

    object_target_pose_ = object_target_pose;

    double targetX = object_target_pose_.position.x;
    double targetY = object_target_pose_.position.y;
    double currentX = getCurrentPose().pose.position.x; 
    double currentY = getCurrentPose().pose.position.y;

    calculated_pose_.yaw = atan2(targetY - currentY, targetX - currentX);

    // Figure out position
    double distance = 1.0;

    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;
    double deltaMagnitude = sqrt(deltaX * deltaX + deltaY * deltaY);

    double unitX = deltaX / deltaMagnitude;
    double unitY = deltaY / deltaMagnitude;

    calculated_pose_.position.x = currentX + unitX * (deltaMagnitude - distance);
    calculated_pose_.position.y = currentY + unitY * (deltaMagnitude - distance);
    calculated_pose_.position.z = fluid::Core::positionFollowHeight;
}

bool fluid::PositionFollowState::hasFinishedExecution() {
    return false;
}

void fluid::PositionFollowState::tick() {
    setpoint.type_mask = fluid::TypeMask::Default;

    if (!has_target_) {
        if (!set_standby_position_) {
            calculated_pose_.position.x = getCurrentPose().pose.position.x;
            calculated_pose_.position.y = getCurrentPose().pose.position.y;
            calculated_pose_.position.z = getCurrentPose().pose.position.z;

            tf2::Quaternion quat(getCurrentPose().pose.orientation.x, 
                                 getCurrentPose().pose.orientation.y, 
                                 getCurrentPose().pose.orientation.z, 
                                 getCurrentPose().pose.orientation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            calculated_pose_.yaw = yaw;

            set_standby_position_ = true;
        }
    }

    setpoint.position.x = calculated_pose_.position.x;
    setpoint.position.y = calculated_pose_.position.y;
    setpoint.position.z = calculated_pose_.position.z;

    // Check first if a boundry is defined (!= 0). If there is a boundry the position target is clamped to 
    // min and max.
    if (fluid::Core::minX != 0 || fluid::Core::maxX != 0) {
        setpoint.position.x = std::max(fluid::Core::minX, 
                                              std::min(calculated_pose_.position.x, fluid::Core::maxX));
    }

    if (fluid::Core::minY != 0 || fluid::Core::maxY != 0) { 
        setpoint.position.y = std::max(fluid::Core::minY, 
                                              std::min(calculated_pose_.position.y, fluid::Core::maxY));
    }

    if (fluid::Core::minZ != 0 || fluid::Core::maxZ != 0) {
        setpoint.position.z = std::max(fluid::Core::minZ, 
                                              std::min(calculated_pose_.position.z, fluid::Core::maxZ));
    }

    setpoint.yaw	= calculated_pose_.yaw;
}