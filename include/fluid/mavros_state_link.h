
#ifndef MAVROS_STATE_LINK_H
#define MAVROS_STATE_LINK_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

/**
 * \brief Handles communication regarding setting and retrieving states within the pixhawk.
 */
class MavrosStateLink
{

private:
    ros::NodeHandle node_handle;
    ros::Subscriber state_subscriber; ///< Retrieves the state changes within the PX4
    mavros_msgs::State current_state;
    ros::ServiceClient set_mode_client; ///< Issues the state change commands
    ros::Time last_request_time = ros::Time::now();

    void stateCallback(const mavros_msgs::State::ConstPtr &msg);

public:
    MavrosStateLink();
    mavros_msgs::State getCurrentState();

    /**
     * Checks if the current state is the state we want the Pixhawk to be in. If not, this function will issue
     * a state change command at the FSM's predefined refresh rate. 
     *
     * @param mode               The mode to attempt to set.
     * @param completion_handler Called when the state change call is responded, will return with a flag whether
     *                           the state change succeeded or not.
     */
    void attemptToSetState(std::string mode, std::function<void(bool)> completion_handler);
};

#endif
