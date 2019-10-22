#include "controller.h"
#include "util.h"
#include "type_mask.h"

mavros_msgs::PositionTarget fluid::Controller::getSetpoint(const ControllerType preferred_controller, 
                                                           const double& time, 
                                                           const std::vector<std::vector<double>>& spline) const {

    mavros_msgs::PositionTarget setpoint;

    ControllerType current_controller = controller_type;

    if (preferred_controller != ControllerType::Passthrough) {
        current_controller = preferred_controller;
    }

    switch (current_controller) {

        case ControllerType::Passthrough:

            ROS_FATAL("The default controller is set to passthrough, defaulting to position.");

        case ControllerType::Positional:
 
            setpoint.position.x = Util::evaluatePolynomial(time, spline[0]);
            setpoint.position.y = Util::evaluatePolynomial(time, spline[1]);
            setpoint.position.z = Util::evaluatePolynomial(time, spline[2]);
            setpoint.yaw = std::atan2(setpoint.position.y, setpoint.position.x);
            setpoint.type_mask = TypeMask::Position;
            break;

        case ControllerType::Velocity:

            setpoint.velocity.x = Util::derivePolynomial(time, spline[0]);
            setpoint.velocity.y = Util::derivePolynomial(time, spline[1]);
            setpoint.velocity.z = Util::derivePolynomial(time, spline[2]);
            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = std::atan2(setpoint.velocity.y, setpoint.velocity.x);
            setpoint.coordinate_frame = 1;
        break;
    }

    return setpoint;
}
