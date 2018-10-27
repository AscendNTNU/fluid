//
// Created by simengangstad on 26.10.18.
//

#include "../../include/mavros/mavros_pose_publisher.h"

void fluid::MavrosPosePublisher::publish(geometry_msgs::PoseStamped pose_stamped) {
    local_position_publisher_.publish(pose_stamped);
}