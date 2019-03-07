//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_STATE_DEFINES_H
#define FLUID_FSM_STATE_DEFINES_H

#include "../core/state.h"
#include <string>

namespace fluid {
    namespace StateIdentifier {
        const std::string     	         Init      = "init",
                                         Idle      = "idle",
                                         TakeOff   = "take_off",
                                         Hold      = "hold",
                                         Move      = "move",
                                         Land      = "land";
    }

    namespace PX4 {
    	const std::string                Offboard = "OFFBOARD",
    				 					 Land 	  = "AUTO.LAND";
    }
}

#endif //FLUID_FSM_STATE_DEFINES_H
