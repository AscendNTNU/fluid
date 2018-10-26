//
// Created by simengangstad on 25.10.18.
//

#ifndef FLUID_FSM_MOVE_OPERATION_H
#define FLUID_FSM_MOVE_OPERATION_H

#include "../core/operation/operation.h"
#include "../core/state.h"
#include "../core/transition.h"

namespace fluid {

    /**
     * \class MoveOperation
     * \brief Encapsulates the operation of moving from a to b.
     */
    class MoveOperation: public Operation {

    private:

        std::vector<std::shared_ptr<fluid::State>>  plan;            ///< Represents the plan of the move operation

        // Methods overridden from superclass Operation
        void stateBegan(const fluid::State& sender);
        void stateFinished(const fluid::State& sender);
        void completed(const fluid::TransitionError transition_error);

    public:

    };
}

#endif //FLUID_FSM_MOVE_OPERATION_H
