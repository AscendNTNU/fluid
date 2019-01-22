//
// Created by simengangstad on 08.01.19.
//

#include "../../include/navigator/navigator_pose_publisher.h"

void fluid::NavigatorPosePublisher::publish(mavros_msgs::PositionTarget position_target) {
    local_position_publisher_.publish(position_target);
}