#ifndef FLUID_CONTROLLER_H
#define FLUID_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <mavros_msgs/PositionTarget.h>
namespace fluid {

    enum class ControllerType {
        // Passthrough will use the current controller 
        Positional, Velocity, Passthrough
    };

    class Controller {

        public:

           ControllerType controller_type = ControllerType::Positional;

           mavros_msgs::PositionTarget getSetpoint(const ControllerType preferred_controller, 
                                                   const double& time, 
                                                   const std::vector<std::vector<double>>& spline) const;
    };
}

#endif