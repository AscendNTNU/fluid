//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_state_setter.h"

void fluid::MavrosStateSetter::setMode(std::string mode) {
    mode_ = mode;
    set_mode_.request.custom_mode = mode_;
}

void fluid::MavrosStateSetter::update(std::function<void (void)> completion_handler) {
    if (state_subscriber_.current_state.mode != mode_ &&
        (ros::Time::now() - last_request_ > ros::Duration(update_interval_))) {

        if (set_mode_client_.call(set_mode_) && set_mode_.response.mode_sent) {
            completion_handler();
        }

        last_request_ = ros::Time::now();
    }
}