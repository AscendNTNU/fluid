/**
 * @file land_state.h
 */

#ifndef LAND_STATE_H
#define LAND_STATE_H

#include "mavros_interface.h"
#include "state.h"
#include "util.h"

/**
 * @brief Represents the state of landing at the current position.
 */
class LandState : public State {
   private:
    /**
     * @return true When the drone is under a certain height and has a certain low velocity.
     */
    bool isBelowThreshold() const;

   public:
    /**
     * @brief Sets up the land state.
     */
    explicit LandState();

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
