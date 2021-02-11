/**
 * @file travel_operation.h
 */

#ifndef TRAVEL_OPERATION_H
#define TRAVEL_OPERATION_H

#include "move_operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the operation where the drone is moving quickly at large distances.
 */
class TravelOperation : public MoveOperation {
   public:
    /**
     * @brief Sets up the travel operation.
     *
     * @param path List of setpoints.
     */
    TravelOperation(const std::vector<geometry_msgs::Point>& path)
        : MoveOperation(OperationIdentifier::TRAVEL, path, 20, 2, 3) {}
};

#endif