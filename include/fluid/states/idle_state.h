/**
 * @file idle_state.h
 */

#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "state.h"

/** 
 * @brief Represents the state where the drone is on ground, armed and spinning its rotors
 */
class IdleState : public State {
   private:
    /**
     * @brief The amount of time to stay idle before saying we're finished executing the state. 
     * 
     */
    const ros::Duration HALT_INTERVAL = ros::Duration(1.0);

    /**
     * @brief The time the idle state starts up. Makes it possible to halt at the idle state for an interval to
     *         make sure that we for example are stable at ground after a land before we do something else.
     */
    ros::Time initial_time;

   public:
    /**
    * @brief Sets up the idle state.
    */
    explicit IdleState() : State(StateIdentifier::Idle, PX4StateIdentifier::Offboard, true, false) {}

    /**
     * @return true When the #HALT_INTERVAL has passed since entering the state.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Sets up the timer for the idle state specified by #initial_time.
     */
    void initialize() override;
};

#endif
