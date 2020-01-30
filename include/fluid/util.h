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

    static std::vector<geometry_msgs::Point> createPath(const geometry_msgs::Point& first, const geometry_msgs::Point& last, const double& density) {
        double distance = distanceBetween(first, last);

        std::vector<geometry_msgs::Point> path;

        float delta_x = last.x - first.x;
        float delta_y = last.y - first.y;
        float delta_z = last.z - first.z;

        for (int i = 0; i < int(density * distance)+1; i++) {
            geometry_msgs::Point temp;

            temp.x = first.x + i * delta_x / (density * distance);
            temp.y = first.y + i * delta_y / (density * distance);
            temp.z = first.z + i * delta_z / (density * distance);
            path.insert(path.end(), temp);
        }

        //std::reverse(path.begin(), path.end());
        return path; 
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
