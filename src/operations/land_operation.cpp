//
// Created by simengangstad on 08.11.18.
//

#include "land_operation.h"

fluid::LandOperation::LandOperation(mavros_msgs::PositionTarget position_target) :
        Operation(fluid::OperationIdentifier::Land,
                  fluid::StateIdentifier::Land,
                  fluid::StateIdentifier::Idle,
                  position_target) {}

bool fluid::LandOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) {
    return current_state_ptr->identifier == "hold";
}

