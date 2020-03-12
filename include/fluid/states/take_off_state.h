/**
 * @file take_off_state.h
 */

#ifndef TAKE_OFF_STATE_H
#define TAKE_OFF_STATE_H

#include "state.h"
#include "util.h"

/** 
 *  @brief Will take off at the current position.
 */
class TakeOffState : public State {
   public:
    /**
     * @brief Setpoint for take off height. 
     */
    float height_setpoint;

    /**
     * @brief Sets up the take off state.
     * 
     * @param height_setpoint The take off height.
     */
    explicit TakeOffState(float height_setpoint);

    /**
     * @return true When the drone has taken off.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the setpoint at the current position with a given altitude.
     */
    void initialize() override;
};

#endif
