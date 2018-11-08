//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_TAKE_OFF_OPERATION_H
#define FLUID_FSM_TAKE_OFF_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"
#include "operation_defines.h"
#include <mavros_msgs/PositionTarget.h>

namespace fluid {

    /**
     * \class TakeOffOperation
     * \brief Encapsulates the operation of taking off.
     */
    class TakeOffOperation: public Operation {

    public:

        TakeOffOperation(mavros_msgs::PositionTarget position_target) :
        Operation(fluid::operation_identifiers::TAKE_OFF, "take_off", "hold", position_target) {}

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p);
    };
}

#endif //FLUID_FSM_TAKE_OFF_OPERATION_H
