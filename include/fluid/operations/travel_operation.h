/**
 * @file travel_operation.h
 */

#ifndef TRAVEL_OPERATION_H
#define TRAVEL_OPERATION_H

#include "move_operation.h"
#include "operation_identifier.h"
#include "mavros_interface.h"

/**
 * @brief Represents the operation where the drone is moving quickly at large distances.
 */
class TravelOperation : public MoveOperation {
   public:
    /**
     * @brief Sets up the travel operation.
     *
     * @param path List of setpoints.
     * @param 100 is the travel speed in [m/s].
     * @param 2 means that setpoints count as visited within 2 [m].
     * @param 3 is the maximum speed the drone can have in the setpoint
     *          to mark it as visited [m/s].
     * @param max_angle is the maximum tilt angle of the drone during movement [deg]. 
     *                  This is set in the base.launch file.
     */
    TravelOperation(const std::vector<geometry_msgs::Point>& path)
        : MoveOperation(OperationIdentifier::TRAVEL, path, 100, 2, 3, Fluid::getInstance().configuration.travel_max_angle) {
            MavrosInterface mavros_interface;
            mavros_interface.setParam("WPNAV_ACCEL", 1000);
            ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max acceleration to: " << 1000/100.0 << " m/s2.");
        }
        
};

#endif