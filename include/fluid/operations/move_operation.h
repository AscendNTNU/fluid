//
// Created by simengangstad on 25.10.18.
//

#ifndef FLUID_FSM_MOVE_OPERATION_H
#define FLUID_FSM_MOVE_OPERATION_H

#include "operation.h"

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
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const override;
    };
}

#endif //FLUID_FSM_MOVE_OPERATION_H
