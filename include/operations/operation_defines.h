//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_OPERATION_DEFINES_H
#define FLUID_FSM_OPERATION_DEFINES_H

#include "../operations/operation_defines.h"

namespace fluid {
    namespace operation_identifiers {
        const fluid::OperationIdentifier INIT      = "init_operation",
                                         IDLE      = "idle_operation",
                                         TAKE_OFF  = "take_off_operation",
                                         HOLD      = "hold_operation",
                                         MOVE      = "move_operation",
                                         LAND      = "land_operation";
    }
}

#endif //FLUID_FSM_OPERATION_DEFINES_H
