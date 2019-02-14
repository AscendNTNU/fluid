//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_state_setter.h"


fluid::MavrosStateSetter::MavrosStateSetter(
    unsigned int message_queue_size,
    double update_interval,
    const std::string mode) :
        
    mode_(mode),
    update_interval_(update_interval),
    set_mode_client_(node_handle_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")) {
        
        setMode(mode);
}

void fluid::MavrosStateSetter::setMode(std::string mode) {
    mode_ = mode;
    set_mode_.request.custom_mode = mode_;
}

mavros_msgs::State fluid::MavrosStateSetter::getCurrentState() {
    return state_subscriber_.getCurrentState();
}

void fluid::MavrosStateSetter::attemptToSetState(std::function<void (bool)> completion_handler) {

    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (state_subscriber_.getCurrentState().mode == mode_) {
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