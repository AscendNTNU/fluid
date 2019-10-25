//
// Created by simengangstad on 22.11.18.
//

#ifndef FLUID_FSM_UTIL_H
#define FLUID_FSM_UTIL_H

#include <geometry_msgs/Point.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <ascend_msgs/Spline.h>
#include <vector>

namespace fluid {
    class Util {
    public:
        static double distanceBetween(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
            double delta_x = target.x - current.x;
            double delta_y = target.y - current.y;
            double delta_z = target.z - current.z;

            return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        }

        static double angleBetween(const geometry_msgs::Quaternion& quaternion, const float& yaw_angle) {
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

        static double derivePolynomial(const double& x, const std::vector<double>& coefficients) {
            const unsigned int degree = coefficients.size() - 1;
            double result = 0;

            for (unsigned int i = 0; i < degree; i++) {
                result += (degree - i) * coefficients[i] * std::pow(x, degree - i - 1); 
            } 

            return result;
        }

        static double evaluatePolynomial(const double& x, const std::vector<double>& coefficients) {
            const unsigned int degree = coefficients.size() - 1;
            double result = 0;

            for (unsigned int i = 0; i < coefficients.size(); i++) {
                result += coefficients[i] * std::pow(x, degree - i); 
            } 

            return result;
        }

        static std::vector<ascend_msgs::Spline> getSplineForSetpoint(const geometry_msgs::Point& current_position, 
                                                                     const geometry_msgs::Point& setpoint) {
            ascend_msgs::Spline spline;

            double dx = setpoint.x - current_position.x;
            double dy = setpoint.y - current_position.y;
            double dz = setpoint.z - current_position.z;

            double timestamp = std::sqrt(dx*dx + dy*dy + dz*dz) * 10;

            std::vector<double> x = {0, 0, 0, 0, dx, current_position.x};
            std::vector<double> y = {0, 0, 0, 0, dy, current_position.y};
            std::vector<double> z = {0, 0, 0, 0, dz, current_position.z};

            spline.x.insert(spline.x.end(), x.begin(), x.end());
            spline.y.insert(spline.y.end(), y.begin(), y.end());
            spline.z.insert(spline.z.end(), z.begin(), z.end());
            spline.timestamp = timestamp;

            return {spline};
        }
    };
}

#endif 
