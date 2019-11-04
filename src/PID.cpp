#include "PID.h"

fluid::PID::PID(const double& kp, const double& ki, const double& kd) : kp(kp), ki(ki), kd(kd) {}

double fluid::PID::getActuation(const double& error, const double& delta_time) {
    integrated_error += error * delta_time;
    double actuation = kp * error + ki * integrated_error + kd * (error - previous_error) / delta_time;
    previous_error = error;

    return actuation;
}
