#include "status_publisher.h"
#include "core.h"

StatusPublisher::StatusPublisher() {
    status.armed = 0;
    status.linked_with_px4 = 0;
    status.px4_mode = "none";
    status.current_operation = "none";
    status.current_state = "none";
    status.path = {};

    publisher = node_handle.advertise<ascend_msgs::FluidStatus>("fluid/status/", 1);
}

void StatusPublisher::publish() { publisher.publish(status); }
