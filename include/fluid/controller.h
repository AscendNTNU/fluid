#ifndef FLUID_CONTROLLER_H
#define FLUID_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <mavros_msgs/PositionTarget.h>
#include <ascend_msgs/Spline.h>

namespace fluid {

    enum class ControllerType {
        // Passthrough will use the current controller 
        Racing, Exploration, Passthrough
    };

    class Controller {

        public:

            Controller(const double& target_speed);

            ControllerType controller_type = ControllerType::Exploration;

            mavros_msgs::PositionTarget getSetpoint(const ControllerType preferred_controller, 
                                                    const double& time, 
                                                    const ascend_msgs::Spline& spline) const;
    };
}

#endif