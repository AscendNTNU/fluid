//
// Created by simengangstad on 08.01.19.
//

#include "../../include/navigator/navigator_state.h"

void fluid::NavigatorState::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
	current_position_.pose = pose->pose;
	current_position_.header = pose->header;
}