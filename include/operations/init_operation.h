//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_INIT_OPERATION_H
#define FLUID_FSM_INIT_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"

namespace fluid {

    /**
     * \class InitOperation
     * \brief Encapsulates the operation of initializing and arming the drone.
     */
    class MoveOperation: public Operation {

    public:

        MoveOperation() : Operation("init_operation", "move", "hold") {}

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p);
    };
}

#endif //FLUID_FSM_INIT_OPERATION_H
