//
// Created by simengangstad on 04.10.18.
//

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../include/states/state_util.h"
#include "../../include/core/transition.h"
#include <algorithm>
#include "../../include/core/fluid_fsm.h"

void fluid::Transition::perform() {

    // The source state is the same as the destination state, we're done here!
    if (source_state_p->identifier == destination_state_p->identifier) {
        return;
    }

    ros::Rate rate(refresh_rate_);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter
    std::string mode = fluid::StateUtil::px4ModeForStateIdentifier(destination_state_p->identifier);
    mavros_state_setter_.setMode(mode);

    bool state_is_set = false;

    // Go through the ros loop and try to set the state
    while(ros::ok() && !state_is_set) {
            
        mavros_state_setter_.attemptToSetState([&](bool succeeded) {
            // State set succeeded, break from loop
            state_is_set = succeeded;
            status_publisher.status.px4_mode = mode;
        });

        // Publish poses continuously so PX4 won't complain
        source_state_p->position_target_publisher_p->publish(source_state_p->position_target);

        status_publisher.publish();

        ros::spinOnce();
        rate.sleep();
    }
}
