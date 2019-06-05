#include "status_publisher.h"
#include "core.h"

fluid::StatusPublisher::StatusPublisher() {
    status.armed	     = 0;
    status.linked_with_px4   = 0;
    status.px4_mode	  = "none";
    status.current_operation = "none";
    status.current_state     = "none";
	status.setpoint.position.x = status.setpoint.position.y = status.setpoint.position.z = 0;
    status.min.x = status.min.y = status.min.z = 0;
	status.max.x = status.max.y = status.max.z = 0;

    publisher_p_ = node_handle_.advertise<ascend_msgs::FluidStatus>("fluid/status/", Core::message_queue_size);
}

void fluid::StatusPublisher::publish() {
	publisher_p_.publish(status);
}
