/**
 * @file travel_operation.h
 */

#ifndef TRAVEL_OPERATION_H
#define TRAVEL_OPERATION_H

#include "move_operation.h"
#include "operation_identifier.h"
#include "mavros_interface.h"

#define LOCATION_SCALING_FACTOR 0.011131884502145034

long diff_longitude(long lon1, long lon2){
    if ((int(lon1) & 0x80000000) == (int(lon2) & 0x80000000))
        // common case of same sign
        return lon1 - lon2;
    long dlon = lon1 - lon2;
    if (dlon > 1800000000)
        dlon -= 3600000000;
    else if (dlon < -1800000000)
        dlon += 3600000000;
    return dlon;
}

geometry_msgs::Point global_to_local(geographic_msgs::GeoPointStamped gp){
    geographic_msgs::GeoPointStamped origin = Fluid::getOrigin();
    geometry_msgs::Point local_pose;
    if(origin.position.altitude !=0){
        double R = 6378100; //6371000  https://github.com/ArduPilot/ardupilot/search?q=earth
        // In NE frame: https://github.com/ArduPilot/ardupilot/blob/e9f6a5afdf33899ca94026075c80733616f74732/libraries/AP_Common/Location.cpp#L252
        local_pose.y = (gp.position.latitude*1E7 - origin.position.latitude) * LOCATION_SCALING_FACTOR;
        //local_pose.y = R* math.radians(origin.latitude/1E7-gp.latitude)
        local_pose.x = diff_longitude(origin.position.longitude,gp.position.longitude*1E7) * LOCATION_SCALING_FACTOR * cos(((origin.position.latitude/1E7+gp.position.latitude)/2) * M_PI / 180.0);
        //local_pose.x = R* math.cos(math.radians(gp.latitude))*math.radians(origin.longitude/1E7-gp.longitude)
        local_pose.z = gp.position.altitude - origin.position.altitude;
        printf("gps in local is x: %f\ty: %f\tz: %f\n", local_pose.x, local_pose.y, local_pose.z);
    }
    else{
        ROS_INFO_STREAM("Travel_operation: can't translate from global to local setpoints. Origin not set yet.");
    }
    return local_pose;
}

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