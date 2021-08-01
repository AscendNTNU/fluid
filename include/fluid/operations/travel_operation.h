/**
 * @file travel_operation.h
 */

#ifndef TRAVEL_OPERATION_H
#define TRAVEL_OPERATION_H

#include "move_operation.h"
#include "operation_identifier.h"
#include "mavros_interface.h"


/*
geometry_msgs::Point global_to_local(geometry_msgs::Point gp){
    if(origin){
        local_pose = geometry_msgs::Point();
        R = 6378100 //6371000  https://github.com/ArduPilot/ardupilot/search?q=earth
        # In NE frame: https://github.com/ArduPilot/ardupilot/blob/e9f6a5afdf33899ca94026075c80733616f74732/libraries/AP_Common/Location.cpp#L252
        local_pose.y = (gp.latitude*1E7 - origin.latitude) * LOCATION_SCALING_FACTOR
        #local_pose.y = R* math.radians(origin.latitude/1E7-gp.latitude)
        local_pose.x = diff_longitude(origin.longitude,gp.longitude*1E7) * LOCATION_SCALING_FACTOR * math.cos(((origin.latitude/1E7+gp.latitude)/2) * math.pi / 180.0)
        #local_pose.x = R* math.cos(math.radians(gp.latitude))*math.radians(origin.longitude/1E7-gp.longitude)
        local_pose.z = gp.altitude - origin.altitude
        print("gps in local is x:" + str(local_pose.x) + "\ty:" + str(local_pose.y) + "\tz:" + str(local_pose.z) + "\n")
    }
}
*/
/**
 * @brief Represents the operation where the drone is moving quickly at large distances.
 */
class TravelOperation : public MoveOperation {
   public:
    /**
     * @brief Sets up the travel operation.
     *
     * @param path List of setpoints.
     * @param speed is the travel speed in [m/s].
     * @param position_threshold means that setpoints count as visited within 2 [m].
     * @param 3 is the maximum speed the drone can have in the setpoint
     *          to mark it as visited [m/s].
     * @param max_angle is the maximum tilt angle of the drone during movement [deg]. 
     *                  This is set in the base.launch file.
     */
    TravelOperation(const std::vector<geometry_msgs::Point>& path)
        : MoveOperation(OperationIdentifier::TRAVEL, path, Fluid::getInstance().configuration.travel_speed, 5, 100, Fluid::getInstance().configuration.travel_max_angle) {
            MavrosInterface mavros_interface;
            mavros_interface.setParam("WPNAV_ACCEL", Fluid::getInstance().configuration.travel_accel*100);
            ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max acceleration to: " << Fluid::getInstance().configuration.travel_accel << " m/s2.");
        }     
};

#endif