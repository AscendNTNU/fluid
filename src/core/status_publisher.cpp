#include "../../include/core/status_publisher.h"
#include <ascend_msgs/FluidFsmStatus.h>

fluid::StatusPublisher::StatusPublisher(const unsigned int message_queue_size) {
    status.armed	     = 0;
    status.linked_with_px4   = 0;
    status.px4_mode	  = "none";
    status.current_operation = "none";
    status.current_state     = "none";

    publisher_p_ = node_handle_.advertise<ascend_msgs::FluidFsmStatus>("fluid_fsm/status/", 
																	   message_queue_size);
}

void fluid::StatusPublisher::publish() {

	publisher_p_.publish(status);
}
