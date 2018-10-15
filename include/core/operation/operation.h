//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_OPERATION_H
#define FLUID_FSM_OPERATION_H

#include <memory>
#include <vector>
#include "../state.h"
#include "../transition.h"
#include "state_graph.h"

namespace fluid {
    /** \class Operation
     *  \brief Manages the transitions between mutiple states
     *
     * Stores a vector of states which it runs through when the operation is started
     */
    class Operation: public StateDelegate, public TransitionDelegate, public std::enable_shared_from_this<Operation> {
    private:

        /** Gets fired whenever a state in the operation began performing.
         *
         * @param sender The state which began performing.
         */
        void stateBegan(const State &sender);

        /** Gets fired whenever a state in the operation finished.
         *
         * @param sender The state which finished.
         */
        void stateFinished(const State &sender);

        /**
         * Gets fired when a transition completes.
         *
         * @param transition_error The error in the transition (if any).
         */
        void completed(TransitionError transition_error);

    protected:

        static StateGraph state_graph; ///< Provides the states which the operation can consist of and how to transition
                                       ///< between them

    public:

        /** Performs the operation.
         *
         * Runs through the different states and performs the necessary transitions.
         */
        void perform();
    };
}

#endif //FLUID_FSM_OPERATION_H
