//
// Created by simengangstad on 25.10.18.
//

#ifndef FLUID_FSM_MOVE_OPERATION_H
#define FLUID_FSM_MOVE_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"
#include "operation_identifier.h"
#include "../states/state_identifier.h"
#include <mavros_msgs/PositionTarget.h>

namespace fluid {

    /**
     * \class MoveOperation
     * \brief Encapsulates the operation of moving from a to b.
     */
    class MoveOperation: public Operation {

    public:

        MoveOperation(mavros_msgs::PositionTarget position_target);
        
        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p);
    };
}

#endif //FLUID_FSM_MOVE_OPERATION_H
