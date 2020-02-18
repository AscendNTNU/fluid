#ifndef OPERATION_H
#define OPERATION_H

#include <memory>
#include <string>

#include <geometry_msgs/Point.h>

#include "core.h"
#include "state.h"
#include "state_identifier.h"

/** 
 *  \brief Manages the transitions between multiple states and their execution.
 */
class Operation {
protected:
    const StateIdentifier destination_state_identifier;

    /** The states the operation should transition to after it has carried
     *  out the logic in the destination state. E.g. if destination state
     *  is set to a move state for a move operation, we want the operation
     *  to finish at a position hold state. These states are all steady.
     */
    const std::map<StateIdentifier, StateIdentifier> steady_state_map = {
        {StateIdentifier::Init, StateIdentifier::Idle},
        {StateIdentifier::Idle, StateIdentifier::Idle},
        {StateIdentifier::TakeOff, StateIdentifier::Hold},
        {StateIdentifier::Explore, StateIdentifier::Hold},
        {StateIdentifier::Travel, StateIdentifier::Hold},
        {StateIdentifier::Hold, StateIdentifier::Hold},
        {StateIdentifier::Rotate, StateIdentifier::Hold},
        {StateIdentifier::Land, StateIdentifier::Idle},
        {StateIdentifier::ExtractModule, StateIdentifier::Hold},
        {StateIdentifier::FollowMast, StateIdentifier::Hold}
    };

public:
    std::vector<ascend_msgs::PositionYawTarget> path;

    Operation(const StateIdentifier& destination_state_identifier,
              const std::vector<ascend_msgs::PositionYawTarget>& path);

    virtual ~Operation() {}

    virtual bool validateOperationFromCurrentState(std::shared_ptr<State> current_state_ptr) const;

    /** 
     * Runs through the different states and performs the necessary transitions.
     *
     * @param should_tick Called each tick, makes it possible to check status further up in the pipeline
     *                    and abort operations in the midst of an execution.
     * @param completionHandler Callback function for whether the operation completed or not.
     */
    virtual void perform(std::function<bool(void)> should_tick, std::function<void(bool)> completionHandler);

    void transitionToState(std::shared_ptr<State> state_p) const;

    StateIdentifier getDestinationStateIdentifier() const;
    std::shared_ptr<State> getFinalStatePtr() const;
    std::shared_ptr<State> getCurrentStatePtr() const;
};

#endif
