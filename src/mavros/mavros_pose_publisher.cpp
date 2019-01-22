//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_pose_publisher.h"

void fluid::MavrosPosePublisher::publish(mavros_msgs::PositionTarget position_target) {
    local_position_publisher_.publish(position_target);
}