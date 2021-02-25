/**
 * @file operation_identifier.h
 */

#ifndef OPERATION_IDENTIFIER_H
#define OPERATION_IDENTIFIER_H

#include <map>
#include <string>

const std::string PX4_MODE_OFFBOARD = "GUIDED";
const std::string PX4_MODE_LAND = "LAND"; //cahnged from AUTO.LAND to LAND for ardupilot  

/**
 * @brief Represents the different operations.
 */
enum class OperationIdentifier {
    // Null is here to make the bredth first search possible in operation graph
    TAKE_OFF,
    HOLD,
    EXPLORE,
    TRAVEL,
    LAND,
    INTERACT,
    UNDEFINED
};

/**
 * @brief Retrieves the PX4 mode for a given @p operation_identifier.
 *
 * @param operation_identifier The operation identifier to get the PX4 mode for.
 *
 * @return The PX4 mode.
 */
std::string getPX4ModeForOperationIdentifier(const OperationIdentifier& operation_identifier);

/**
 * @brief Get a string from the @p operation_identifier.
 *
 * @param operation_identifier The operation identifier.
 *
 * @return The operation identifier represented in a string.
 */
std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier);

#endif
