/**
 * @file take_off_operation.h
 */

#ifndef TAKE_OFF_OPERATION_H
#define TAKE_OFF_OPERATION_H

#include "operation.h"
#include "util.h"

/**
 *  @brief Will take off at the current position.
 */
class TakeOffOperation : public Operation {
   public:
    /**
     * @brief Setpoint for take off height.
     */
    float height_setpoint;

    /**
     * @brief Sets up the take off operation.
     *
     * @param height_setpoint The take off height.
     */
    explicit TakeOffOperation(float height_setpoint);

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
