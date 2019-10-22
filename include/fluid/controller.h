#ifndef FLUID_CONTROLLER_H
#define FLUID_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <mavros_msgs/PositionTarget.h>
namespace fluid {

    enum class ControllerType {
        Positional, Velocity
    };

    class Controller {

        public:

           fluid::ControllerType controller_type = ControllerType::Positional;
           mavros_msgs::PositionTarget getSetpoint(const double& time, const std::vector<std::vector<double>>& spline) const;
    };
}

#endif