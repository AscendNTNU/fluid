//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_TAKE_OFF_OPERATION_H
#define FLUID_FSM_TAKE_OFF_OPERATION_H

#include "operation.h"

namespace fluid {

    /**
     * \class TakeOffOperation
     * \brief Encapsulates the operation of taking off.
     */
    class TakeOffOperation: public Operation {

    public:

        TakeOffOperation(mavros_msgs::PositionTarget position_target);

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const override;
    };
}

#endif //FLUID_FSM_TAKE_OFF_OPERATION_H
