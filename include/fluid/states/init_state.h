/**
 * @file init_state.h
 */

#ifndef INIT_STATE_H
#define INIT_STATE_H

#include "state.h"

/**
 * @brief Makes sure everything is initialized (link to mavros and px4) before any further transitions are called.
 */
class InitState : public State {
   private:
   public:
    /**
    * @brief Initializes the init state. Will not issue go through the tick method from #State as this class
    *        performs all its logic in #initialize.
    */
    explicit InitState() : State(StateIdentifier::Init, PX4StateIdentifier::Offboard, false, false) {}

    /**
     * @brief Sets up the connection with PX4, requests arm and offboard.
     */
    void initialize() override;

    /**
     * @return true When the state has finished execution.
     */
    bool hasFinishedExecution() const override;
};

#endif
