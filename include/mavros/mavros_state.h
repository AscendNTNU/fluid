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
         * @param should_check_obstacle_avoidance Whether we should check obstacle avoidance completion in this state.
         */
        MavrosState(std::string identifier, std::string px4_mode, bool should_check_obstacle_avoidance_completion);
    };
}

#endif //FLUID_FSM_MAVROS_STATE_H
