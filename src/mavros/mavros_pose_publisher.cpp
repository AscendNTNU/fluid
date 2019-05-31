//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_pose_publisher.h"

fluid::MavrosPosePublisher::MavrosPosePublisher(unsigned int message_queue_size) :
    local_position_publisher_(node_handle_.advertise<mavros_msgs::PositionTarget>("fluid_fsm/setpoint",
                                                                                  message_queue_size)) {}


void fluid::MavrosPosePublisher::publish(mavros_msgs::PositionTarget position_target) {
	position_target.header.stamp = ros::Time::now();
    local_position_publisher_.publish(position_target);
}