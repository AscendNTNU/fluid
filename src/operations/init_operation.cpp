//
// Created by simengangstad on 08.11.18.
//

#include "../../include/operations/init_operation.h"

bool fluid::MoveOperation::validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) {
    return current_state_p->identifier == "init";
}
