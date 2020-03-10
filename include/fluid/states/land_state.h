/**
 * @file land_state.h
 */

#ifndef LAND_STATE_H
#define LAND_STATE_H

#include "state.h"
#include "util.h"

/** 
 * @brief Represents the state of landing at the current position. 
 */
class LandState : public State {
   public:
    /**
     * @brief Sets up the land state.
     */
    explicit LandState() : State(StateIdentifier::Land, PX4StateIdentifier::Land, false, false) {}

    /**
     * @return true When the drone has landed.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the setpoint to the current position with zero altitude.
     */
    void initialize() override;
};

#endif
