//
// Created by simengangstad on 08.11.18.
//

#include "../../include/fluid/operations/take_off_operation.h"


fluid::TakeOffOperation::TakeOffOperation(mavros_msgs::PositionTarget position_target) :
								          Operation(fluid::OperationIdentifier::TakeOff,
								                    fluid::StateIdentifier::TakeOff,
								                    fluid::StateIdentifier::Hold,
								                    position_target) {

	// We want that the take off is executed from the current position at ground, so we
	// grab the current x and y values.
	this->position_target.position.x = fluid::Core::getGraphPtr()->current_state_p->getCurrentPose().pose.position.x;
	this->position_target.position.y = fluid::Core::getGraphPtr()->current_state_p->getCurrentPose().pose.position.y;

	fluid::Core::getStatusPublisherPtr()->status.target_pose_x = this->position_target.position.x;
	fluid::Core::getStatusPublisherPtr()->status.target_pose_y = this->position_target.position.y;
}

bool fluid::TakeOffOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "idle";
}