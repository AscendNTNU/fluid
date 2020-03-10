/**
 * @file hold_state.h
 */

#ifndef HOLD_STATE_H
#define HOLD_STATE_H

#include "state.h"
#include "util.h"

/** 
 * @brief State representing the drone hovering. 
 */
class HoldState : public State {
   public:
    /**
     * @brief Sets up the hold state. 
     */
    explicit HoldState() : State(StateIdentifier::Hold, PX4StateIdentifier::Offboard, true, false) {}

    /**
     * @return true When the drone is hovering still at a given position.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the setpoint to the current position.
     */
    void initialize() override;
};

#endif
