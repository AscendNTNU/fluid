#ifndef UTIL_H
#define UTIL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <vector>

class Util {
public:
    static double distanceBetween(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
        double delta_x = target.x - current.x;
        double delta_y = target.y - current.y;
        double delta_z = target.z - current.z;

        return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    static double angleBetween(const geometry_msgs::Quaternion& quaternion, const float& yaw_angle) {
        tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
        // it to zero.
        yaw = std::isnan(yaw) ? 0.0 : yaw;
        const auto yaw_error = yaw_angle - yaw;

        return std::atan2(std::sin(yaw_error), std::cos(yaw_error));
    }

    static double clampAngle(double angle) { return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI; }
};

#endif
