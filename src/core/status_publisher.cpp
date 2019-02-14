#include "../../include/core/status_publisher.h"
#include <ascend_msgs/FluidFsmStatus.h>
#include "../../include/core/core.h"

fluid::StatusPublisher::StatusPublisher() {
    status.armed	     = 0;
    status.linked_with_px4   = 0;
    status.px4_mode	  = "none";
    status.current_operation = "none";
    status.current_state     = "none";
    status.target_pose_x = 0;
	status.target_pose_y = 0;
	status.target_pose_z = 0;
	status.half_boundry_x = 0;
	status.half_boundry_y = 0;
	status.half_boundry_z = 0;

    publisher_p_ = node_handle_.advertise<ascend_msgs::FluidFsmStatus>("fluid_fsm/status/", Core::message_queue_size);
}

void fluid::StatusPublisher::publish() {

	node_handle_.getParam("boundryX", status.half_boundry_x);
	node_handle_.getParam("boundryY", status.half_boundry_y);
	node_handle_.getParam("boundryZ", status.half_boundry_z);

	publisher_p_.publish(status);
}
