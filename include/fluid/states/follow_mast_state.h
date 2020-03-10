/**
 * @file follow_mast_state.h
 */

#ifndef FOLLOW_MAST_STATE_H
#define FOLLOW_MAST_STATE_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "state.h"

/**
 * @brief Represents the state where the drone is following the module.
 */
class FollowMastState : public State {
   private:
    /**
     * @brief The module position.
     */
    geometry_msgs::PoseWithCovarianceStamped module_info;

    /**
     * @brief Grabs the module position.
     */
    ros::Subscriber module_position_subscriber;

    /**
     * @brief Updates the module position.
     * 
     * @param module_position The new module position. 
     */
    void modulePositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position);

   public:
    /**
     * @brief Sets up the follow mast state.
     */
    explicit FollowMastState();

    /**
     * @return Will return indefinitely until a state change is requested.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Updates the setpoint in correspondance with the module position.
     */
    void tick() override;
};

#endif
