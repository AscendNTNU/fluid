//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_MAVROS_STATE_H
#define FLUID_FSM_MAVROS_STATE_H

#include "../core/state.h"
#include "../core/core.h"

namespace fluid {
    /**
     * \class MavrosState
     * \brief Encapsulates a state which publishes poses through mavros.
     */
    class MavrosState: public State {
        
    public:

        /**
         * Initiializes the mavros state with an identifier.
         * 
         * @param identifier The identifier of the state.
         * @param px4_mode The mode this mavros state represents within px4.
         */
        MavrosState(std::string identifier, std::string px4_mode);
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
