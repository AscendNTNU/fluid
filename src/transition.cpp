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

        fluid::Core::getStatusPublisherPtr()->status.current_state = destination_state_p->identifier;
        fluid::Core::getStatusPublisherPtr()->status.path = destination_state_p->path;

        fluid::Core::getGraphPtr()->current_state_ptr = destination_state_p;
        
        return;
    }

    ros::Rate rate(Core::refresh_rate);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter

    bool state_is_set = false;

    fluid::Core::getStatusPublisherPtr()->status.current_state = source_state_p->identifier;
    fluid::Core::getGraphPtr()->current_state_ptr = source_state_p;

    // Go through the ros loop and try to set the state
    while(ros::ok() && !state_is_set) {

        // Publish poses continuously so PX4 won't complain, have to have tick here so the type mask is 
        // set up correctly
        source_state_p->publishSetpoint();

        fluid::Core::getStatusPublisherPtr()->status.path = source_state_p->path;
        fluid::Core::getStatusPublisherPtr()->publish();

        mavros_state_link_.attemptToSetState(destination_state_p->px4_mode, [&](bool succeeded) {
            // State set succeeded, break from loop

            destination_state_p->current_pose = source_state_p->getCurrentPose();
            state_is_set = succeeded;
            
            fluid::Core::getStatusPublisherPtr()->status.current_state = destination_state_p->identifier;
            fluid::Core::getStatusPublisherPtr()->status.path = destination_state_p->path;
            fluid::Core::getStatusPublisherPtr()->status.px4_mode = destination_state_p->px4_mode;

            fluid::Core::getGraphPtr()->current_state_ptr = destination_state_p;
        });

       
        ros::spinOnce();
        rate.sleep();
    }
}
