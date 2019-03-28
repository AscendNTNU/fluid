//
// Created by simengangstad on 27.10.18.
//

#include "../../include/mavros/mavros_state.h"
#include "../../include/core/core.h"
#include "../../include/mavros/mavros_pose_publisher.h"

#include <utility>
#include <memory>


fluid::MavrosState::MavrosState(std::string identifier, std::string px4_mode) : 
	State(std::move(identifier),
	px4_mode, 
	"mavros/local_position/pose",
	"mavros/local_position/velocity", 
    std::make_shared<fluid::MavrosPosePublisher>(Core::message_queue_size)) {}