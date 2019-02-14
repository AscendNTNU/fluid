//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/take_off_operation.h"


fluid::TakeOffOperation::TakeOffOperation(mavros_msgs::PositionTarget position_target) :
								          Operation(fluid::operation_identifiers::TAKE_OFF,
								                    fluid::StateIdentifiers::TAKE_OFF,
								                    fluid::StateIdentifiers::HOLD,
								                    position_target) {}

bool fluid::TakeOffOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "idle";
}
