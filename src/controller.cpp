#include "controller.h"
#include "util.h"
#include "type_mask.h"

fluid::Controller::Controller(const fluid::ControllerType& controller_type) : controller_type(controller_type) {}

mavros_msgs::PositionTarget fluid::Controller::getSetpoint(const double& time, std::shared_ptr<std::vector<std::vector<float>>> spline_ptr) const {

    mavros_msgs::PositionTarget setpoint;

    switch (controller_type) {
        case ControllerType::Positional:
 
            setpoint.position.x = Util::evaluatePolynomial(time, (*spline_ptr)[0]);
            setpoint.position.y = Util::evaluatePolynomial(time, (*spline_ptr)[1]);
            setpoint.position.z = Util::evaluatePolynomial(time, (*spline_ptr)[2]);
            setpoint.yaw = std::atan2(setpoint.position.y, setpoint.position.x);
            setpoint.type_mask = TypeMask::Position;
            break;

        case ControllerType::Velocity:

            setpoint.velocity.x = Util::derivePolynomial(time, (*spline_ptr)[0]);
            setpoint.velocity.y = Util::derivePolynomial(time, (*spline_ptr)[1]);
            setpoint.velocity.z = Util::derivePolynomial(time, (*spline_ptr)[2]);
            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = std::atan2(setpoint.velocity.y, setpoint.velocity.x);
            setpoint.coordinate_frame = 1;
        break;
    }

    return setpoint;
}
