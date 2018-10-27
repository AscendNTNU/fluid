//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_state_subscriber.h"

void fluid::MavrosStateSubscriber::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
