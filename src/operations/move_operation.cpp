//
// Created by simengangstad on 26.10.18.
//

#include "move_operation.h"

fluid::MoveOperation::MoveOperation(mavros_msgs::PositionTarget position_target) :
					    		    Operation(fluid::OperationIdentifier::Move,
							                  fluid::StateIdentifier::Move,
							                  fluid::StateIdentifier::Hold,
							                  position_target) {}

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr)const {
    return current_state_ptr->identifier == "hold" || current_state_ptr->identifier == "move";
}

