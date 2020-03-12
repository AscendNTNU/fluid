/**
 * @file state_identifier.h
 */

#ifndef STATE_IDENTIFIER_H
#define STATE_IDENTIFIER_H

#include <map>
#include <string>
#include <memory>

#include "init_state.h"
#include "idle_state.h"
#include "take_off_state.h"
#include "hold_state.h"
#include "explore_state.h"
#include "travel_state.h"
#include "land_state.h"
#include "extract_module_state.h"
#include "follow_mast_state.h"

const std::string PX4_MODE_OFFBOARD = "OFFBOARD";
const std::string PX4_MODE_LAND = "MODE";

/**
 * @brief Represents the different states.
 */
enum class StateIdentifier {
    // Null is here to make the bredth first search possible in state graph
    INIT,
    IDLE,
    TAKE_OFF,
    HOLD,
    EXPLORE,
    TRAVEL,
    LAND,
    EXTRACT_MODULE,
    FOLLOW_MAST,
    UNDEFINED
};

/**
 * @brief Retrieves the PX4 mode for a given @p state_identifier. 
 * 
 * @param state_identifier The state identifier to get the PX4 mode for. 
 * 
 * @return The PX4 mode.
 */
std::string getPX4ModeForStateIdentifier(const StateIdentifier& state_identifier) {
    switch (state_identifier) {
        case StateIdentifier::LAND:
            return PX4_MODE_LAND;
        default:
            return PX4_MODE_OFFBOARD;
    }
}

/**
 * @brief Get a string from the @p state_identifier. 
 * 
 * @param state_identifier The state identifier. 
 * 
 * @return The state identifier represented in a string.
 */
std::string getStringFromStateIdentifier(const StateIdentifier& state_identifier) {
    switch (state_identifier) {
        case StateIdentifier::INIT:
            return "INIT";
        case StateIdentifier::IDLE:
            return "IDLE";
        case StateIdentifier::TAKE_OFF:
            return "TAKE_OFF";
        case StateIdentifier::HOLD:
            return "HOLD";
        case StateIdentifier::EXPLORE:
            return "EXPLORE";
        case StateIdentifier::TRAVEL:
            return "TRAVEL";
        case StateIdentifier::LAND:
            return "LAND";
        case StateIdentifier::EXTRACT_MODULE:
            return "EXTRACT_MODULE";
        case StateIdentifier::FOLLOW_MAST:
            return "FOLLOW_MAST";
        case StateIdentifier::UNDEFINED:
            return "UNDEFINED";
    }
}

/**
 * @brief Constructs a state pointer from the @p state_identifier. 
 * 
 * @param state_identifier The state identifier to construct the state from.
 * 
 * @return The state pointer. 
 */
std::shared_ptr<State> constructStateFromStateIdentifier(const StateIdentifier& state_identifier) {
    switch (state_identifier) {
        case StateIdentifier::INIT:
            return std::make_shared<InitState>();
        case StateIdentifier::IDLE:
            return std::make_shared<IdleState>();
        case StateIdentifier::TAKE_OFF:
            return std::make_shared<TakeOffState>();
        case StateIdentifier::HOLD:
            return std::make_shared<HoldState>();
        case StateIdentifier::EXPLORE:
            return std::make_shared<ExploreState>();
        case StateIdentifier::TRAVEL:
            return std::make_shared<TravelState>();
        case StateIdentifier::LAND:
            return std::make_shared<LandState>();
        case StateIdentifier::EXTRACT_MODULE:
            return std::make_shared<ExtractModuleState>();
        case StateIdentifier::FOLLOW_MAST:
            return std::make_shared<FollowMastState>();
        case StateIdentifier::UNDEFINED:
            return nullptr;
    }
}

#endif
