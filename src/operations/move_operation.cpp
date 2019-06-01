//
// Created by simengangstad on 26.10.18.
//

#include "../../include/fluid/operations/move_operation.h"


fluid::MoveOperation::MoveOperation(mavros_msgs::PositionTarget position_target) :
					    		    Operation(fluid::OperationIdentifier::Move,
							                  fluid::StateIdentifier::Move,
							                  fluid::StateIdentifier::Hold,
							                  position_target) {}

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "hold" || current_state_p->identifier == "move";
}

