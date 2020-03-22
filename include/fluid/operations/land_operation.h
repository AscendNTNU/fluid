/**
 * @file land_operation.h
 */

#ifndef LAND_OPERATION_H
#define LAND_OPERATION_H

#include "mavros_interface.h"
#include "operation.h"
#include "util.h"

/**
 * @brief Represents the operation of landing at the current position.
 */
class LandOperation : public Operation {
   private:
    /**
     * @return true When the drone is under a certain height and has a certain low velocity.
     */
    bool isBelowThreshold() const;

   public:
    /**
     * @brief Sets up the land operation.
     */
    explicit LandOperation();

    /**
     * @return true When the drone has landed.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the setpoint to the current position with zero altitude.
     */
    void initialize() override;

    /**
     * @brief Checks if the drone isBelowThreshold() and will set setpoint type mask to #TypeMask::IDLE so that it
     *        stays idle at ground.
     */
    void tick() override;
};

#endif
