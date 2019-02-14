//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_STATE_DEFINES_H
#define FLUID_FSM_STATE_DEFINES_H

#include "../core/state.h"
#include <string>

namespace fluid {
    namespace StateIdentifiers {
        const fluid::StateIdentifier     INIT      = "init",
                                         IDLE      = "idle",
                                         TAKE_OFF  = "take_off",
                                         HOLD      = "hold",
                                         MOVE      = "move",
                                         LAND      = "land";
    }

    namespace PX4 {
    	const std::string                OFFBOARD = "OFFBOARD",
    				 					 LAND 	  = "AUTO.LAND";
    }
}

#endif //FLUID_FSM_STATE_DEFINES_H
