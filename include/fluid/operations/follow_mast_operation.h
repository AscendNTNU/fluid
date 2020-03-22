/**
 * @file follow_mast_operation.h
 */

#ifndef FOLLOW_MAST_OPERATION_H
#define FOLLOW_MAST_OPERATION_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "operation.h"

/**
 * @brief Represents the operation where the drone is following the module.
 */
class FollowMastOperation : public Operation {
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
     * @brief Sets up the follow mast operation.
     */
    explicit FollowMastOperation();

    /**
     * @return Will return indefinitely until a operation change is requested.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Updates the setpoint in correspondance with the module position.
     */
    void tick() override;
};

#endif
