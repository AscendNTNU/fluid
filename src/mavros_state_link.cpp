#include "mavros_state_link.h"
#include "core.h"

MavrosStateLink::MavrosStateLink()
    : state_subscriber(
          node_handle.subscribe<mavros_msgs::State>("mavros/state", 1, &MavrosStateLink::stateCallback, this)),
      set_mode_client(node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")) {}

void MavrosStateLink::stateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

mavros_msgs::State MavrosStateLink::getCurrentState() { return current_state; }

void MavrosStateLink::attemptToSetState(std::string mode, std::function<void(bool)> completion_handler) {
    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (getCurrentState().mode == mode) {
        completion_handler(true);
    } else {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (ros::Time::now() - last_request_time > ros::Duration(1.0 / static_cast<double>(Core::refresh_rate))) {
            if (set_mode_client.call(set_mode)) {
                completion_handler(set_mode.response.mode_sent);
            }

            last_request_time = ros::Time::now();
        }
    }
}