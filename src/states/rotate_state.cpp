#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include "core.h"
#include "rotate_state.h"
#include "util.h"

bool RotateState::hasFinishedExecution() const {
    bool atPositionTarget = Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) <
                                Core::distance_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.x) < Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.y) < Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;

    bool atYawTarget =
        std::abs(Util::angleBetween(getCurrentPose().pose.orientation, setpoint.yaw)) < Core::yaw_completion_threshold;

    return atYawTarget && atPositionTarget;
}

void RotateState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z;
    setpoint.type_mask = TypeMask::Position;

    double dx = path.front().point.x - getCurrentPose().pose.position.x;
    double dy = path.front().point.y - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);
}
