#include "status_publisher.h"
#include "core.h"

fluid::StatusPublisher::StatusPublisher() {
    status.armed	     = 0;
    status.linked_with_px4   = 0;
    status.px4_mode	  = "none";
    status.current_operation = "none";
    status.current_state     = "none";
    status.target_pose_x = 0;
	status.target_pose_y = 0;
	status.target_pose_z = 0;
	status.min_x = 0;
	status.min_y = 0;
	status.min_z = 0;
	status.max_x = 0;
	status.max_y = 0;
	status.max_z = 0;

    publisher_p_ = node_handle_.advertise<ascend_msgs::FluidFsmStatus>("fluid_fsm/status/", Core::message_queue_size);
}

void fluid::StatusPublisher::publish() {
	publisher_p_.publish(status);
}
