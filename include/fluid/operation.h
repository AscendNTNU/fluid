#ifndef FLUID_FSM_OPERATION_H
#define FLUID_FSM_OPERATION_H

#include <memory>
#include <string>

#include <geometry_msgs/Point.h>

#include "state.h"
#include "core.h"
#include "state_identifier.h"

namespace fluid {

    /** \class Operation
     *  \brief Manages the transitions between multiple states and their execution.
     */
    class Operation {
    protected:

        const std::string destination_state_identifier_;                        

        /** The states the operation should transition to after it has carried
         *  out the logic in the destination state. E.g. if destination state
         *  is set to a move state for a move operation, we want the operation
         *  to finish at a position hold state. These states are all steady.
         */
        const std::map<std::string, std::string> steady_state_map_ = {
            {fluid::StateIdentifier::Init, fluid::StateIdentifier::Idle},
            {fluid::StateIdentifier::Idle, fluid::StateIdentifier::Idle},
            {fluid::StateIdentifier::TakeOff, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Move, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Hold, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::PositionFollow, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Rotate, fluid::StateIdentifier::Hold},
            {fluid::StateIdentifier::Land, fluid::StateIdentifier::Idle},
        };



        public:

        std::vector<geometry_msgs::Point> path;                

        Operation(const std::string& destination_state_identifier,
                  const std::vector<geometry_msgs::Point>& path);


        virtual ~Operation() {}

        virtual bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_ptr) const;

        /** 
         * Runs through the different states and performs the necessary transitions.
         *
         * @param tick Called each tick, makes it possible to check status further up in the pipeline
         *             and abort operations in the midst of an execution.
         * @param completionHandler Callback function for whether the operation completed or not.
         */
        virtual void perform(std::function<bool (void)> tick, std::function<void (bool)> completionHandler);

        void transitionToState(std::shared_ptr<fluid::State> state_p);

        std::string getDestinationStateIdentifier() const;
        std::shared_ptr<fluid::State> getFinalStatePtr() const;
        std::shared_ptr<fluid::State> getCurrentStatePtr() const;
    };
}

#endif
