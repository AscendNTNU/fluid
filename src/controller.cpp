#include "controller.h"
#include "util.h"
#include "type_mask.h"

fluid::Controller::Controller(const std::string& topic, const unsigned int& degree) : topic_(topic), degree_(degree) {
    setpoint_publisher_ = node_handle_.advertise<mavros_msgs::PositionTarget>(topic_, 10);
}

void fluid::Controller::tick(std::shared_ptr<std::vector<std::vector<float>>> spline_ptr) const {

    double x, y, z;
    x = y = z = 0;

    if (degree_ == 1) {
        x = Util::derivePolynomial(ros::Time().toSec(), (*spline_ptr)[0]);
        y = Util::derivePolynomial(ros::Time().toSec(), (*spline_ptr)[1]);
        z = Util::derivePolynomial(ros::Time().toSec(), (*spline_ptr)[2]);
    }

    // TODO: Implement for different degrees

    double yaw = std::atan2(y, x);

    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Velocity;
    setpoint.velocity.x = x;
    setpoint.velocity.y = y;
    setpoint.velocity.z = z;

    setpoint_publisher_.publish(setpoint);
}