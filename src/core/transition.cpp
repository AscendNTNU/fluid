//
// Created by simengangstad on 04.10.18.
//

#include "transition.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "core.h"

fluid::Transition::Transition(std::shared_ptr<State> source_state_p, std::shared_ptr<State> destination_state_p) :
                              source_state_p(std::move(source_state_p)),
                              destination_state_p(std::move(destination_state_p)) {}

void fluid::Transition::perform() {

    // The source state is the same as the destination state, we're done here!
    if (source_state_p->identifier == destination_state_p->identifier) {
        return;
    }

    if (source_state_p->px4_mode == destination_state_p->px4_mode) {
        return;
    }

    ros::Rate rate(Core::refresh_rate);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter

    bool state_is_set = false;

    // Go through the ros loop and try to set the state
    while(ros::ok() && !state_is_set) {
            
        mavros_state_link_.attemptToSetState(destination_state_p->px4_mode, [&](bool succeeded) {
            // State set succeeded, break from loop

            destination_state_p->setCurrentPose(source_state_p->getCurrentPose());
            state_is_set = succeeded;
            
            fluid::Core::getStatusPublisherPtr()->status.setpoint = destination_state_p->setpoint;
            fluid::Core::getStatusPublisherPtr()->status.px4_mode = destination_state_p->px4_mode;
        });

        // Publish poses continuously so PX4 won't complain, have to have tick here so the type mask is 
        // set up correctly
        source_state_p->tick();
        source_state_p->setpoint_publisher.publish(source_state_p->setpoint);

        fluid::Core::getStatusPublisherPtr()->status.setpoint = source_state_p->setpoint;
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();
    }
}
