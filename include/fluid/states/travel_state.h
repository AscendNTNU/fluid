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
     * 
     * @param path List of setpoints.
     */
    TravelState(const std::vector<geometry_msgs::Point>& path) : MoveState(StateIdentifier::TRAVEL, path, 20, 20, 20) {}
};

#endif