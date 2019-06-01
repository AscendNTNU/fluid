//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/mavros_state_link.h"
#include "../../include/core/core.h"

fluid::MavrosStateLink::MavrosStateLink() : 
    state_subscriber_(node_handle_.subscribe<mavros_msgs::State>("mavros/state",
                                                                 Core::message_queue_size,
                                                                 &MavrosStateLink::stateCallback,
                                                                 this)), 
    set_mode_client_(node_handle_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")) {}

void fluid::MavrosStateLink::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

mavros_msgs::State fluid::MavrosStateLink::getCurrentState() {
    return current_state_;
}

void fluid::MavrosStateLink::attemptToSetState(std::string mode, std::function<void (bool)> completion_handler) {

    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (getCurrentState().mode == mode) {
        completion_handler(true);
    }
    else {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (ros::Time::now() - last_request_ > ros::Duration(1.0/static_cast<double>(Core::refresh_rate))) {

            if (set_mode_client_.call(set_mode)) {
                completion_handler(set_mode.response.mode_sent);
            }

            last_request_ = ros::Time::now();
        }
    }
}