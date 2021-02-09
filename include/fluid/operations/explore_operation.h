/**
 * @file explore_operation.h
 */

#ifndef EXPLORE_OPERATION_H
#define EXPLORE_OPERATION_H

#include <ascend_msgs/ObstacleAvoidanceService.h>
#include <ascend_msgs/Path.h>

#include "move_operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the a move operation where the drone is following a path and avoiding obstacles.
 */
class ExploreOperation : public MoveOperation {
   private:
    /**
     * @brief The density of the path passed to obstacle avoidance.
     *
     */
    const double path_density = 4;

    /**
     * @brief The path filled with points at a #path_density, passed to obstacle avoidance.
     */
    std::vector<geometry_msgs::Point> dense_path;

    /**
     * @brief Publishes the current path to obstacle avoidance.
     */
    ros::Publisher obstacle_avoidance_path_publisher;

    /**
     * @brief Grabs the path from obstacle avoidance which is altered to avoid obstacles.
     */
    ros::Subscriber obstacle_avoidance_path_subscriber;

    /**
     * @brief The original path the operation was initialized with.
     */
    std::vector<geometry_msgs::Point> original_path;

    /**
     * @brief The corrected path from obstacle avoidance.
     */
    std::vector<geometry_msgs::Point> corrected_path;

    /**
     * @brief Determines if the original path got set.
     */
    bool original_path_set = false;

    /**
     * @brief Point of interest from the end of the original path.
     */
    geometry_msgs::Point point_of_interest;

    /**
     * @brief Callback for the corrected path from obstacle avoidance.
     *
     * @param corrected_path The corrected path.
     */
    void pathCallback(ascend_msgs::Path corrected_path);

   public:
    /**
     * @brief Sets up the explore operation.
     *
     * @param path List of setpoints.
     */
    explicit ExploreOperation(const std::vector<geometry_msgs::Point>& path);

    /**
     * @brief Sets up the #dense_path.
     */
    void initialize() override;

    /**
     * @brief Publishes the #dense_path to obstacle avoidance.
     */
    void tick() override;
};

#endif
