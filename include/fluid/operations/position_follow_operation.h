//
// Created by simengangstad on 07.03.19.
//

#ifndef FLUID_FSM_POSITION_FOLLOW_OPERATION_H
#define FLUID_FSM_POSITION_FOLLOW_OPERATION_H

#include "operation.h"

namespace fluid {

    /**
     * \class PositionFollowOperation
     * \brief Encapsulates the operation of following after a point, e.g. a person.
     */
    class PositionFollowOperation: public Operation {

    public:

        PositionFollowOperation();

        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) override;
    };
}

#endif
