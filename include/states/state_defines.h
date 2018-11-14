//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_STATE_DEFINES_H
#define FLUID_FSM_STATE_DEFINES_H

namespace fluid {
    namespace state_identifiers {
        const fluid::StateIdentifier     INIT      = "init",
                                         IDLE      = "idle",
                                         TAKE_OFF  = "take_off",
                                         HOLD      = "hold",
                                         MOVE      = "move",
                                         LAND      = "land";
    }
}

#endif //FLUID_FSM_STATE_DEFINES_H
