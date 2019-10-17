#include "racing_controller.h"
#include "util.h"
#include "type_mask.h"

fluid::RacingController::RacingController(const std::string& topic, const unsigned int& degree) : Controller(topic, degree) {}


void fluid::RacingController::tick(std::shared_ptr<std::vector<std::vector<float>>> spline) const {

    double velocity_x = Util::derivePolynomial(ros::Time().toSec(), (*spline)[0]);
    double velocity_y = Util::derivePolynomial(ros::Time().toSec(), (*spline)[1]);
    double velocity_z = Util::derivePolynomial(ros::Time().toSec(), (*spline)[2]);

    double yaw = std::atan2(velocity_y, velocity_x);

    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Velocity;
    setpoint.velocity.x = velocity_x;
    setpoint.velocity.y = velocity_y;
    setpoint.velocity.z = velocity_z;

    setpoint_publisher_.publish(setpoint);
}