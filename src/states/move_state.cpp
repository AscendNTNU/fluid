#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/ParamSet.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include "core.h"
#include "move_state.h"
#include "util.h"

bool MoveState::hasFinishedExecution() const { return been_to_all_points; }

void MoveState::tick() {
    bool at_position_target =
        Util::distanceBetween(getCurrentPose().pose.position, *current_destination_point_iterator) < position_threshold;
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;

    if (at_position_target && low_enough_velocity) {
        if (current_destination_point_iterator < path.end() - 1) {
            current_destination_point_iterator++;
            setpoint.position = *current_destination_point_iterator;

            double dx = current_destination_point_iterator->x - getCurrentPose().pose.position.x;
            double dy = current_destination_point_iterator->y - getCurrentPose().pose.position.y;
            setpoint.yaw = std::atan2(dy, dx);
        } else {
            been_to_all_points = true;
        }
    }
}

void MoveState::initialize() {
    for (auto iterator = path.begin(); iterator != path.end(); iterator++) {
        if (iterator->z <= 0.1) {
            iterator->z = Core::default_height;
        }
    }

    setpoint.type_mask = TypeMask::Position;
    been_to_all_points = false;
    current_destination_point_iterator = path.begin();
    setpoint.position = *current_destination_point_iterator;

    double dx = current_destination_point_iterator->x - getCurrentPose().pose.position.x;
    double dy = current_destination_point_iterator->y - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);

    ros::ServiceClient param_set_service_client = node_handle.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

    mavros_msgs::ParamSet param_set_service;

    param_set_service.request.param_id = "MPC_XY_VEL_MAX";
    param_set_service.request.value.real = speed;
    ROS_INFO_STREAM("Setting speed to: " << speed);
    if (!param_set_service_client.call(param_set_service)) {
        ROS_FATAL_STREAM("Could not set the MPC_XY_VEL_MAX for PX4.");
    }
}
