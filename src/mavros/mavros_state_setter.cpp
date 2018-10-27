//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_state_setter.h"

void fluid::MavrosStateSetter::setMode(std::string mode) {
    mode_ = mode;
    set_mode_.request.custom_mode = mode_;
}

void fluid::MavrosStateSetter::attemptToSetState(std::function<void (bool)> completion_handler) {

    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (state_subscriber_.current_state.mode == mode_) {
        completion_handler(true);
    }
    else {
        if (ros::Time::now() - last_request_ > ros::Duration(update_interval_)) {

            if (set_mode_client_.call(set_mode_)) {
                completion_handler(set_mode_.response.mode_sent);
            }

            last_request_ = ros::Time::now();
        }
    }
}