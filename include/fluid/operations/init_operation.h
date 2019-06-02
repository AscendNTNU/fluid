//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_INIT_OPERATION_H
#define FLUID_FSM_INIT_OPERATION_H

#include "operation.h"

namespace fluid {

    /**
     * \class InitOperation
     * \brief Encapsulates the operation of initializing and arming the drone.
     */
    class InitOperation: public Operation {

    public:

        InitOperation(mavros_msgs::PositionTarget position_target);

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const override;
    };
}

#endif //FLUID_FSM_INIT_OPERATION_H
