//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_OPERATION_H
#define FLUID_FSM_OPERATION_H

#include <memory>
#include <string>

#include <mavros_msgs/PositionTarget.h>

#include "state.h"
#include "core.h"
#include "state_identifier.h"

namespace fluid {

    /** \class Operation
     *  \brief Manages the transitions between multiple states and their execution.
     */
    class Operation {
    protected:

        const std::string destination_state_identifier_;                        ///< The state the operation should
                                                                                ///< transition to.

        /** The states the operation should transition to after it has carried
         *  out the logic in the destination state. E.g. if destination state
         *  is set to a move state for a move operation, we want the operation
         *  to finish at a position hold state. These states are all steady.
         */
        const std::map<std::string, std::string> final_state_map_ = {
            {fluid::StateIdentifier::Init, fluid::StateIdentifier::Idle},
            {fluid::StateIdentifier::Idle, fluid::StateIdentifier::Idle},
            {fluid::StateIdentifier::TakeOff, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Move, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Hold, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::PositionFollow, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Land, fluid::StateIdentifier::Idle},
        };



        public:

        mavros_msgs::PositionTarget position_target;                ///< Position target of the operation.

        /**
         * Sets up the operation. 
         *
         * @param destination_state_identifier   The destination state identifier of the operation.
         * @param position_target                The target position of this operation.
         */
        Operation(std::string destination_state_identifier,
                  mavros_msgs::PositionTarget position_target);


        virtual ~Operation() {}

        /**
         * Checks if the operation is valid from the current state. 
         * 
         * @param current_state_p       The current state.
         *
         * @return A flag determining the validation of the operation given the current state.
         */
        virtual bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const;

        /** 
         * Performs the operation.
         *
         * Runs through the different states and performs the necessary transitions.
         *
         * @param shouldAbort Called each tick, makes it possible to abort operations in the midst of an execution.
         * @param completionHandler Callback function for whether the operation completed or not.
         */
        virtual void perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler);

        /**
         * @brief Sets the pose for a new state and performs the transition to that state from the current state.
         * 
         * This function also sets the current state of the state graph to the state passed as the argument.
         */
        void transitionToState(std::shared_ptr<fluid::State> state_p);

        /**
         * @return The destination state this operation will traverse to.
         */
        std::string getDestinationStateIdentifier() const;

        /**
         * @return The state the operation should end at.
         */
        std::shared_ptr<fluid::State> getFinalStatePtr() const;

        /**
         * @return The current state of the operation.
         */
        std::shared_ptr<fluid::State> getCurrentStatePtr() const;
    };
}

#endif //FLUID_FSM_OPERATION_H
