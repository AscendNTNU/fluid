//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_LAND_OPERATION_H
#define FLUID_FSM_LAND_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"

namespace fluid {

    /**
     * \class LandOperation
     * \brief Encapsulates the operation of landing at the current position.
     */
    class MoveOperation: public Operation {

    public:

        MoveOperation() : Operation("land_operation", "land", "idle") {}

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p);
    };
}

#endif //FLUID_FSM_LAND_OPERATION_H
