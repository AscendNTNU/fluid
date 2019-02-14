//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_state_subscriber.h"
#include "../../include/core/core.h"

fluid::MavrosStateSubscriber::MavrosStateSubscriber() {

	state_subscriber = node_handle_.subscribe<mavros_msgs::State>("mavros/state",
					                                              Core::message_queue_size,
					                                              &MavrosStateSubscriber::state_callback,
																  this);
}

void fluid::MavrosStateSubscriber::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    
    current_state = *msg;
}

mavros_msgs::State fluid::MavrosStateSubscriber::getCurrentState() {
    return current_state;
}