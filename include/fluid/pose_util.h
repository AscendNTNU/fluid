//
// Created by simengangstad on 22.11.18.
//

#ifndef FLUID_FSM_POSE_UTIL_H
#define FLUID_FSM_POSE_UTIL_H

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

namespace fluid {
    class PoseUtil {
    public:
        static double distanceBetween(geometry_msgs::PoseStamped current, mavros_msgs::PositionTarget target) {
            double delta_x = target.position.x - current.pose.position.x;
            double delta_y = target.position.y - current.pose.position.y;
            double delta_z = target.position.z - current.pose.position.z;

            return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        }

        static double angleBetween(geometry_msgs::Quaternion quaternion, float yaw_angle) {
            tf2::Quaternion quat(quaternion.x, 
                                 quaternion.y, 
                                 quaternion.z, 
                                 quaternion.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
            // it to zero. 
            yaw = std::isnan(yaw) ? 0.0 : yaw;
            const auto yaw_error = yaw_angle - yaw;

            return std::atan2(std::sin(yaw_error), std::cos(yaw_error)); 
        }  
    };
}

#endif //FLUID_FSM_POSE_UTIL_H
