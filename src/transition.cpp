#include "transition.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "core.h"
#include "state_identifier.h"

Transition::Transition(std::shared_ptr<State> source_state_ptr, std::shared_ptr<State> destination_state_ptr)
    : source_state_ptr(std::move(source_state_ptr)), destination_state_ptr(std::move(destination_state_ptr)) {}

void Transition::perform() {
    // The source state is the same as the destination state, we're done here!
    if (source_state_ptr->identifier == destination_state_ptr->identifier) {
        return;
    }

    if (source_state_ptr->px4_mode == destination_state_ptr->px4_mode) {
        Core::getStatusPublisherPtr()->status.current_state =
            StateIdentifierStringMap.at(destination_state_ptr->identifier);
        Core::getStatusPublisherPtr()->status.path = destination_state_ptr->path;

        Core::getGraphPtr()->current_state_ptr = destination_state_ptr;

        return;
    }

    ros::Rate rate(Core::refresh_rate);

    // Get the px4 mode for the state we want to transition to and set that mode in our state setter

    bool state_is_set = false;

    Core::getStatusPublisherPtr()->status.current_state = StateIdentifierStringMap.at(source_state_ptr->identifier);
    Core::getGraphPtr()->current_state_ptr = source_state_ptr;

    // Go through the ros loop and try to set the state
    while (ros::ok() && !state_is_set) {
        // Publish poses continuously so PX4 won't complain, have to have tick here so the type mask is
        // set up correctly
        source_state_ptr->publishSetpoint();

        Core::getStatusPublisherPtr()->status.path = source_state_ptr->path;
        Core::getStatusPublisherPtr()->publish();

        mavros_state_link.attemptToSetState(
            PX4StateIdentifierStringMap.at(destination_state_ptr->px4_mode), [&](bool succeeded) {
                // State set succeeded, break from loop

                destination_state_ptr->current_pose = source_state_ptr->getCurrentPose();
                state_is_set = succeeded;

                Core::getStatusPublisherPtr()->status.current_state =
                    StateIdentifierStringMap.at(destination_state_ptr->identifier);
                Core::getStatusPublisherPtr()->status.path = destination_state_ptr->path;
                Core::getStatusPublisherPtr()->status.px4_mode =
                    PX4StateIdentifierStringMap.at(destination_state_ptr->px4_mode);

                Core::getGraphPtr()->current_state_ptr = destination_state_ptr;
            });

        ros::spinOnce();
        rate.sleep();
    }
}
