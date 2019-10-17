#include "controller.h"

fluid::Controller::Controller(const std::string& topic, const unsigned int& degree) : topic_(topic), degree_(degree) {
    setpoint_publisher_ = node_handle_.advertise<mavros_msgs::PositionTarget>(topic_, 10);
}