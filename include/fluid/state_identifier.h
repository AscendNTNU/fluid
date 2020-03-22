/**
 * @file state_identifier.h
 */

#ifndef STATE_IDENTIFIER_H
#define STATE_IDENTIFIER_H

#include <map>
#include <string>

const std::string PX4_MODE_OFFBOARD = "OFFBOARD";
const std::string PX4_MODE_LAND = "AUTO.LAND";

/**
 * @brief Represents the different states.
 */
enum class StateIdentifier {
    // Null is here to make the bredth first search possible in state graph
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
std::string getPX4ModeForStateIdentifier(const StateIdentifier& state_identifier);

/**
 * @brief Get a string from the @p state_identifier.
 *
 * @param state_identifier The state identifier.
 *
 * @return The state identifier represented in a string.
 */
std::string getStringFromStateIdentifier(const StateIdentifier& state_identifier);

/**
 * @brief Constructs a state pointer from the @p state_identifier.
 *
 * @param state_identifier The state identifier to construct the state from.
 *
 * @return The state pointer.
 */

#endif
