/**
 * @file explore_state.h
 */

#ifndef EXPLORE_STATE_H
#define EXPLORE_STATE_H

#include <ascend_msgs/ObstacleAvoidanceService.h>
#include <ascend_msgs/Path.h>
#include "move_state.h"
#include "state_identifier.h"

/**
 * @brief Represents the a move state where the drone is following a path and avoiding obstacles.
 */
class ExploreState : public MoveState {
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
     * @brief The original path the state was initialized with.
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
     * @brief Callback for the corrected path from obstacle avoidance.
     * 
     * @param corrected_path The corrected path. 
     */
    void pathCallback(ascend_msgs::Path corrected_path);

   public:
    /**
     * @brief Sets up the explore state.
     * 
     * @param path List of setpoints.
     */
    explicit ExploreState(const std::vector<geometry_msgs::Point>& path);

    /**
     * @brief Sets up the #dense_path.
     */
    void initialize() override;

    /**
     * @brief Publishes the #dense_path to obstacle avoidance.
     */
    void tick() override;

    /**
     * @brief Resets the flag for #original_path_set.
     */
    void finalize() override;
};

#endif
