//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_TRANSITION_H
#define FLUID_FSM_TRANSITION_H

#include <iostream>
#include <memory>
#include "state.h"


// TODO: Check the errors which can come from state changes in the FSM in PX4
/** \enum TransitionErrorCode
 *  \brief Describes which kind of error which happened in a given transition.
 */
enum TransitionErrorCode: int8_t {
    no_error = 0,
    px4_error = 1
};

/** \struct TransitionError
 *  \brief Describes the error which happened in the transitions between two states.
 */
struct TransitionError {
    TransitionErrorCode error_code;     ///< The code which specifies which transition error occurred
    std::string start_state;            ///< The start state of the transition
    std::string end_state;              ///< The end state of the transition
};

class Transition;

/** \class TransitionDelegate
 *  \brief Interface for callbacks from transitions.
 *
 *  Implement this and set yourself as the delegate for the given transition to receive a completion callback with
 *  possible errors. These errors are coming from the FSM in PX4 if there are any. E.g. an invalid state change.
 */
class TransitionDelegate {
public:

    /**
     * Called when the transition completed.
     *
     * @param transition_error The error from the transition (if any). If the transition went as expected the error code
     *                         will be 0.
     */
    virtual void completed(TransitionError transition_error) = 0;
};


/** \class Transition
 *  \breif Handles state changes and the communication with the FSM in PX4.
 */
class Transition {
private:
    const std::shared_ptr<State> start_state_p_;                    ///< Start state
    const std::shared_ptr<State> end_state_p_;                      ///< End state

public:
    std::weak_ptr<TransitionDelegate> transition_delegate_p; ///< The delegate which receive completion callback

    /**
     * Initializes a transition with start and end states.
     *
     * @param start_state_p The start state (pass a shared pointer)
     * @param end_state_p The end state (pass a shared pointer)
     */
    Transition(std::shared_ptr<State> start_state_p,
               std::shared_ptr<State> end_state_p) : start_state_p_(start_state_p), end_state_p_(end_state_p) {}

    /**
     * Performs the transition between the start state and the end state.
     */
    void perform();
};


#endif //FLUID_FSM_TRANSITION_H
