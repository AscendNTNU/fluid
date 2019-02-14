//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/init_operation.h"

fluid::InitOperation::InitOperation(mavros_msgs::PositionTarget position_target) :
        							Operation(fluid::operation_identifiers::INIT,
							                  fluid::StateIdentifiers::INIT,
							                  fluid::StateIdentifiers::IDLE,
							                  position_target) {}


bool fluid::InitOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "init";
}
