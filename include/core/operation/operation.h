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
     *  \brief Manages the transitions between multiple states
     *
     * Stores a vector of states which it runs through when the operation is started
     */
    class Operation {
    private:

        std::shared_ptr<fluid::State> destination_state_p_;                     ///< The state the operation should
                                                                                ///< transition to.

        std::shared_ptr<fluid::State> final_state_p_;                           ///< The state the operation should
                                                                                ///< transition to after it has carried
                                                                                ///< out the logic in the destination
                                                                                ///< state. E.g. if destination state
                                                                                ///< is set to a move state for a move
                                                                                ///< operation, we want the operation
                                                                                ///< to finish at a position hold state.

        static StateGraph state_graph;                              ///< Provides the states which the operation can
                                                                    ///< consist of and how to transition between them

    public:


        /**
         * Sets up the operation with a destination state and a final state. The difference between them is that the
         * destination state is the state the operation will transition to and carry out logic on, whereas final state
         * is the state we want to be at after the operation. E.g. a move operation would want to be at a final state
         * of position hold after a given move state.
         *
         * @param destination_state_p   The destination state of the operation.
         * @param final_state_p         The final state.
         */
        Operation(ros::NodeHandlePtr node_handle_p,
                  std::shared_ptr<fluid::State> destination_state_p,
                  std::shared_ptr<fluid::State> end_state_p) :

                  destination_state_p_(destination_state_p_),
                  final_state_p_(final_state_p_) {}


        /**
         * Checks if the operation is valid from the current state. This makes sure that some operations are not
         * carried out given that they make no sense from the current state. E.g. doing any operation before
         * everything is initialized or doing a land operation during take off.
         *
         * @param current_state_p       The current state.
         *
         * @return A flag determining the validation of the operation given the current state.
         */
        virtual bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> destination_state_p) = 0;

        /** Performs the operation.
         *
         * Runs through the different states and performs the necessary transitions.
         *
         * @param completion_handler Callback function for whether the operation completed or not.
         */
         // TODO: Some sort of error attached in the callback?
        void perform(std::function<void (bool)> completion_handler);
    };
}

#endif //FLUID_FSM_OPERATION_H
