//
// Created by simengangstad on 07.03.19.
//

#include "../../include/fluid/operations/position_follow_operation.h"

fluid::PositionFollowOperation::PositionFollowOperation() : Operation(fluid::OperationIdentifier::PositionFollow,
													                  fluid::StateIdentifier::PositionFollow,
													                  fluid::StateIdentifier::Hold,
													                  mavros_msgs::PositionTarget()) {}


bool fluid::PositionFollowOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const {
    return current_state_ptr->identifier == "hold" || current_state_ptr->identifier == "move";
}
