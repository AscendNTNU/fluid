#ifndef EXPLORE_STATE_H
#define EXPLORE_STATE_H

#include <ascend_msgs/ObstacleAvoidanceService.h>
#include <ascend_msgs/Path.h>
#include "move_state.h"
#include "state_identifier.h"
#include <visualization_msgs/Marker.h>

/**
 * \brief Drone is following a path and (optionally) facing towards a certain point
 */
class ExploreState : public MoveState {
private:
    std::vector<geometry_msgs::Point> dense_path;
    const double path_density = 4;
    bool retrieved_point_of_interest = false;
    geometry_msgs::Point point_of_interest;
    ros::Subscriber point_of_interest_subscriber;
    ros::Publisher obstacle_avoidance_path_publisher;
    ros::Subscriber obstacle_avoidance_path_subscriber;
    ros::Publisher current_setpoint_visualization_publisher;
    std::vector<ascend_msgs::PositionYawTarget> original_path;
    std::vector<ascend_msgs::PositionYawTarget> corrected_path;

    bool original_path_set = false;

    void pointOfInterestCallback(const geometry_msgs::PointConstPtr& point);

public:
    ExploreState()
        : MoveState(StateIdentifier::Explore, 0.3, 0.3, 0.3),
          obstacle_avoidance_path_publisher(node_handle.advertise<ascend_msgs::Path>("/obstacle_avoidance/path", 10)),
          obstacle_avoidance_path_subscriber(node_handle.subscribe("/obstacle_avoidance/corrected_path", 10, &ExploreState::pathCallback, this)),
          current_setpoint_visualization_publisher(node_handle.advertise<visualization_msgs::Marker>("/fluid/setpoint_visualiztion", 10)) {}

    void pathCallback(ascend_msgs::Path corrected_path);

    void initialize() override;
    void tick() override;
    void finalize() override;
};

#endif
