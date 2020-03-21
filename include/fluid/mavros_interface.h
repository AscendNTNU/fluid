/**
 * @file mavros_state_link.h
 */

#ifndef MAVROS_STATE_LINK_H
#define MAVROS_STATE_LINK_H

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

/**
 * @brief Handles communication regarding setting state, retriving state from the pixhawk, as well as
 *        convenience functions for arming, offboard mode and setting parameters.
 */
class MavrosInterface {
   private:
    /**
    * @brief How fast the mavros interface will check for state changes when setting new modes in PX4.
    */
    const unsigned int UPDATE_REFRESH_RATE = 5;

    /**
     * @brief Retrieves the state changes within PX4.
     */
    ros::Subscriber state_subscriber;

    /**
     * @brief Current state of PX4.
     */
    mavros_msgs::State current_state;

    /**
     * @brief Publishes setpoints.
     */
    ros::Publisher setpoint_publisher;

    /**
     * @brief Callback for the state within PX4. 
     * 
     * @param msg The state message. 
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

   public:
    /**
    * @brief Sets up the required subscribers and service clients.
    */
    MavrosInterface();

    /**
     * @return The current state gotten from PX4 through mavros.
     */
    mavros_msgs::State getCurrentState() const;

    /**
     * @brief Sets up the connection with PX4 through MAVROS.
     */
    void establishContactToPX4() const;

    /**
     * @brief Will attempt to set the @p mode if PX4 is not already in the given mode. 
     *
     * @param mode               The mode to attempt to set.
     * 
     * @return true if the mode got set.
     */
    bool attemptToSetState(const std::string& mode) const;

    /**
     * @brief Requests PX4 to arm.
     * 
     * @param auto_arm Will arm automatically if set to true, if not it'll wait until an arm signal is 
     *                 retrieved from the RC.
     */
    void requestArm(const bool& auto_arm) const;

    /**
     * @brief Requests PX4 to go into offboard mode.
     * 
     * @param auto_offboard Will go into offboard mode automatically if set to true, if not it'll wait until
     *                      offboard flight mode is set from RC. 
     */
    void requestOffboard(const bool& auto_offboard) const;

    /**
     * @brief Sets a parameter within PX4.
     * 
     * @param parameter The parameter to set.
     * @param value The new value. 
     */
    void setParam(const std::string& parameter, const float& value) const;
};

#endif
