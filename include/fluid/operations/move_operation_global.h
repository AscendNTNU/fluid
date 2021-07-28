/**
 * @file move_operation_global.h
 */

#ifndef MOVE_OPERATION_H
#define MOVE_OPERATION_H

#include "operation.h"
#include <mavros_msgs/GlobalPositionTarget.h>

/**
 * @brief Serves as the base for move operations such as #ExploreOperation and #TravelOperation.
 */
class MoveOperation : public Operation {
   private:
    /**
     * @brief Publisher to global position setpoint.
     */
    ros::Publisher global_pose_pub;

    /**
     * @brief Subscriber to global position setpoint.
     */
    ros::Subscriber global_pose_sub;

    /**
     * @brief The threshold for when the drone is within a given setpoint.
     */
    const double position_threshold;

    /**
     * @brief The highest velocity the drone can have in order to signal that it can move to the next setpoint in the
     *        #path.
     */
    const double velocity_threshold;

    /**
     * @brief The speed the drone should move at.
     */
    const double speed;

    /**
     * @brief Maximum allowed angle during movement.
     */
    const double max_angle;

    /**
     * @brief Convenicene variable representing that the drone has been through all the setpoints in the #path.
     */
    bool been_to_all_points = false;
    
   protected:
    /**
     * @brief Flag for forcing the operation to update the setpoint even the drone hasn't reached the setpoint.
     */
    bool update_setpoint = false;

    /**
     * @brief The current setpoint.
     */
    std::vector<mavros_msgs::GlobalPositionTarget>::iterator current_setpoint_iterator;
 
    /**
     * @brief List of the setpoints.
     */
    std::vector<mavros_msgs::GlobalPositionTarget> path;

    /**
     * @brief Sets up the move operation.
     *
     * @param operation_identifier The operation identifier.
     * @param path The path of the operation.
     * @param speed The speed at which to move in [m/s].
     * @param position_threshold Distance setpoints must be within to count as visited [m].
     * @param velocity_threshold The velocity threshold in [m/s].
     * @param max_angle Is the maximum allowed angle during movement [deg].
     */
    explicit MoveOperation(const OperationIdentifier& operation_identifier,
                           const std::vector<mavros_msgs::GlobalPositionTarget>& path, const double& speed,
                           const double& position_threshold, const double& velocity_threshold,
                           const double& max_angle);

   public:
    /**
     * @return true When the drone has been through the whole #path.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Checks where the drone is at a given point and updates the #current_setpoint_iterator if
     *        the drone has reached a setpoint.
     */
    virtual void tick() override;

    /**
     * @brief Sets up the #current_setpoint_iterator and sets the #speed at which to move.
     */
    virtual void initialize() override;
};

#endif
