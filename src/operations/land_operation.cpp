//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/land_operation.h"

bool fluid::LandOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "move" || current_state_p->identifier == "hold";
}

