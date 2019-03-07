#include "../../include/states/position_follow_state.h"
#include "../../include/mavros/type_mask.h"

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

    object_target_pose_ = object_target_pose;

    double targetX = object_target_pose_.position.x;
    double targetY = object_target_pose_.position.y;
    double currentX = getCurrentPose().pose.position.x; 
    double currentY = getCurrentPose().pose.position.y;

    double theta = atan2(targetY - currentY, targetX - currentX);

    tf2::Quaternion quat(getCurrentPose().pose.orientation.x, 
                         getCurrentPose().pose.orientation.y, 
                         getCurrentPose().pose.orientation.z, 
                         getCurrentPose().pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
    // it to zero. 
    yaw_target = (std::isnan(yaw) ? 0.0 : yaw) - theta;
}

bool fluid::PositionFollowState::hasFinishedExecution() {
    return false;
}

void fluid::PositionFollowState::tick() {
    position_target.type_mask = fluid::TypeMask::Default;

    /*
    
    ROS_FATAL_STREAM(theta * 180.0/3.14159);

    // Figure out position
    double distance = 1.0;

	double deltaX = targetX - currentX;
	double deltaY = targetY - currentY;
    double deltaMagnitude = sqrt(deltaX * deltaX + deltaY * deltaY);

    double unitX = deltaX / deltaMagnitude;
    double unitY = deltaY / deltaMagnitude;

    double newX = currentX + unitX*(deltaMagnitude - distance);
    double newY = currentY + unitY*(deltaMagnitude - distance);

    position_target.position.x = newX;
    position_target.position.y = newY;
    position_target.position.z = 1.5;

    ROS_INFO_STREAM("x: " << position_target.position.x << ", y: " << position_target.position.y << ", z: " << position_target.position.z);*/

    position_target.position.z = 1.5;
    position_target.yaw = yaw_target;
}