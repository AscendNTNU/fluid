//
// Created by simengangstad on 07.03.19.
//

#include "../../include/operations/position_follow_operation.h"

fluid::PositionFollowOperation::PositionFollowOperation() : Operation(fluid::OperationIdentifier::PositionFollow,
													                  fluid::StateIdentifier::PositionFollow,
													                  fluid::StateIdentifier::Idle,
													                  mavros_msgs::PositionTarget()) {}


bool fluid::PositionFollowOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "hold" || current_state_p->identifier == "move";
}
