//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/take_off_operation.h"

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "idle";
}
