//
// Created by simengangstad on 26.10.18.
//

#include "../../include/operations/move_operation.h"


fluid::MoveOperation::MoveOperation(mavros_msgs::PositionTarget position_target) :
					    		    Operation(fluid::operation_identifiers::MOVE,
							                  fluid::StateIdentifiers::MOVE,
							                  fluid::StateIdentifiers::HOLD,
							                  position_target) {}

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "hold" || current_state_p->identifier == "move";
}

