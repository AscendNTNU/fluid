/**
 * @file travel_state.h
 */

#ifndef TRAVEL_STATE_H
#define TRAVEL_STATE_H

#include "move_state.h"
#include "state_identifier.h"

/**
 * @brief Represents the state where the drone is moving quickly at large distances.
 */
class TravelState : public MoveState {
   public:
    /**
     * @brief Sets up the travel state.
     */
    TravelState() : MoveState(StateIdentifier::TRAVEL, 20, 20, 20) {}
};

#endif