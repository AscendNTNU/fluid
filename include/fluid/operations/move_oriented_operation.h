//
// Created by simengangstad on 02.05.19.
//

#ifndef FLUID_FSM_MOVE_ORIENTED_OPERATION_H
#define FLUID_FSM_MOVE_ORIENTED_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"
#include "operation_identifier.h"
#include "../states/state_identifier.h"
#include <mavros_msgs/PositionTarget.h>

namespace fluid {

    /**
     * \class MoveOrientedOperation
     * \brief A move operation where the drone orientates itself towards the setpoint and then moves.
     */
    class MoveOrientedOperation: public Operation {

    public:

        MoveOrientedOperation(mavros_msgs::PositionTarget position_target);
        
        /**
         * Method overriden from superclass.
         */
        bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p);

        void perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler) override;
    };
}

#endif //FLUID_FSM_MOVE_ORIENTED_OPERATION_H
