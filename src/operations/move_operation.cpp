//
// Created by simengangstad on 26.10.18.
//

#include "../../include/operations/move_operation.h"
#include "../../include/core/state.h"
#include <memory>

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "hold" || current_state_p->identifier == "move";
}

