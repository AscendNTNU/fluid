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
	status.min_x = 0;
	status.min_y = 0;
	status.min_z = 0;
	status.max_x = 0;
	status.max_y = 0;
	status.max_z = 0;

    publisher_p_ = node_handle_.advertise<ascend_msgs::FluidFsmStatus>("fluid_fsm/status/", Core::message_queue_size);
}

void fluid::StatusPublisher::publish() {

	node_handle_.getParam("minX", status.min_x);
	node_handle_.getParam("minY", status.min_y);
	node_handle_.getParam("minZ", status.min_z);

	node_handle_.getParam("maxX", status.max_x);
	node_handle_.getParam("maxY", status.max_y);
	node_handle_.getParam("maxZ", status.max_z);

	publisher_p_.publish(status);
}
