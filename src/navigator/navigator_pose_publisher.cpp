//
// Created by simengangstad on 08.01.19.
//

#include "../../include/navigator/navigator_pose_publisher.h"
#include "../../include/core/core.h"

fluid::NavigatorPosePublisher::NavigatorPosePublisher(std::string topic) :
local_position_publisher_(node_handle_.advertise<mavros_msgs::PositionTarget>(topic, Core::message_queue_size)) {}


void fluid::NavigatorPosePublisher::publish(mavros_msgs::PositionTarget position_target) {
    local_position_publisher_.publish(position_target);
}