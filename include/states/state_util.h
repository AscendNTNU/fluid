//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_STATE_UTIL_H
#define FLUID_FSM_STATE_UTIL_H

#include "state_identifier.h"

namespace fluid {
    class StateUtil {
    public:

        static std::string px4ModeForStateIdentifier(fluid::StateIdentifier state_identifier) {
            switch (state_identifier) {
                case fluid::StateIdentifier::idle:
                    return "OFFBOARD";
                case fluid::StateIdentifier::take_off:
                    // Can't use the default AUTO.TAKEOFF mode as we haven't got a GPS on the drone, check this
                    return "OFFBOARD";
                case fluid::StateIdentifier::hold:
                    // Can't use AUTO.LOITER as it requires GPS
                    return "OFFBOARD";
                case fluid::StateIdentifier::move:
                    return "OFFBOARD";
                case fluid::StateIdentifier::land:
                    return "AUTO.LAND";
            }
        }
    };
}

#endif //FLUID_FSM_STATE_UTIL_H
