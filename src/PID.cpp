#include "PID.h"

fluid::PID::PID(const double& kp, const double& ki, const double& kd) : kp(kp), ki(ki), kd(kd) {}

double fluid::PID::getActuation(const double& error, const double& delta_time) {
    integrated_error += error * delta_time;
    double derivative_error = previous_derivative_error * 0.5 + (error - previous_error) * 0.5;
    double actuation = kp * error + ki * integrated_error + kd * derivative_error / delta_time;
    previous_error = error;
    previous_derivative_error = derivative_error;

    return actuation;
}
