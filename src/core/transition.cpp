//
// Created by simengangstad on 04.10.18.
//

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../include/core/transition.h"
#include <algorithm>
#include "../../include/core/core.h"

fluid::Transition::Transition(std::shared_ptr<State> source_state_p, std::shared_ptr<State> destination_state_p) :
                   
    mavros_state_setter_(Core::message_queue_size, 1.0/static_cast<double>(Core::refresh_rate), "OFFBOARD"),
    source_state_p(std::move(source_state_p)),
    destination_state_p(std::move(destination_state_p)) {}

void fluid::Transition::perform() {

    // The source state is the same as the destination state, we're done here!
    if (source_state_p->identifier == destination_state_p->identifier) {
        return;
    }

    ros::Rate rate(Core::refresh_rate);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter
    mavros_state_setter_.setMode(destination_state_p->px4_mode);

    bool state_is_set = false;

    // Go through the ros loop and try to set the state
    while(ros::ok() && !state_is_set) {
            
        mavros_state_setter_.attemptToSetState([&](bool succeeded) {
            // State set succeeded, break from loop
            state_is_set = succeeded;
            Core::getStatusPublisherPtr()->status.px4_mode = destination_state_p->px4_mode;
        });

        // Publish poses continuously so PX4 won't complain
        source_state_p->position_target_publisher_p->publish(source_state_p->position_target);

        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();
    }
}
