//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/init_operation.h"

fluid::InitOperation::InitOperation(mavros_msgs::PositionTarget position_target) :
        							Operation(fluid::OperationIdentifier::Init,
							                  fluid::StateIdentifier::Init,
							                  fluid::StateIdentifier::Idle,
							                  position_target) {}


bool fluid::InitOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "init";
}
