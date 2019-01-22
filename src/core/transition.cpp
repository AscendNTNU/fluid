//
// Created by simengangstad on 04.10.18.
//

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../include/states/state_util.h"
#include "../../include/core/transition.h"
#include <algorithm>

void fluid::Transition::perform(std::function<void (void)> completion_handler) {

    // The source state is the same as the destination state, we're done here!
    if (source_state_p->identifier == destination_state_p->identifier) {
        completion_handler();
        return;
    }

    TransitionErrorCode transition_error_code = no_error;

    ros::Rate rate(refresh_rate_);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter
    std::string mode = fluid::StateUtil::px4ModeForStateIdentifier(destination_state_p->identifier);
    mavros_state_setter_.setMode(mode);

    bool state_is_set = false;

    // Go through the ros loop and try to set the state
    while(ros::ok() && !state_is_set) {
            
        mavros_state_setter_.attemptToSetState([&](bool succeeded) {
            // State set succeeded, break from loop
            //ROS_INFO_STREAM("Transitioning from " << source_state_p->identifier << " -> " << destination_state_p->identifier);
            //ROS_INFO_STREAM("Attempt to set mode " << mode.c_str() << ", succeeded:" << succeeded);
            state_is_set = succeeded;
        });

        // Publish poses continuously so PX4 won't complain
        source_state_p->position_target_publisher_p->publish(source_state_p->position_target);

        ros::spinOnce();
        rate.sleep();
    }

    TransitionError transition_error = {transition_error_code,
                                        source_state_p->identifier,
                                        destination_state_p->identifier};

    // TODO: Currently we don't pass a error (if any) here, figure out which error we can get from the px4 state change
    // TODO: Though will the loop try to set the state until it succeeds, so it might not be necessary
    completion_handler();
}
