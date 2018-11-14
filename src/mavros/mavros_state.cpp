//
// Created by simengangstad on 27.10.18.
//

#include "../../include/mavros/mavros_state.h"

void fluid::MavrosState::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_position_.pose = pose->pose;
    current_position_.header = pose->header;
}