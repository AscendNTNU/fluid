#ifndef EXPLORE_STATE_H
#define EXPLORE_STATE_H

#include <ascend_msgs/ObstacleAvoidanceService.h>
#include "move_state.h"
#include "state_identifier.h"

/**
 * \brief Drone is following a path and (optionally) facing towards a certain point
 */
class ExploreState : public MoveState {
private:
    bool retrieved_point_of_interest = false;
    geometry_msgs::Point point_of_interest;
    ros::Subscriber point_of_interest_subscriber;
    ros::ServiceClient obstacle_avoidance_service_client;
    std::vector<geometry_msgs::Point> original_path;

    void pointOfInterestCallback(const geometry_msgs::PointConstPtr& point);

public:
    ExploreState()
        : MoveState(StateIdentifier::Explore, 0.3, 0.3, 0.3),
          obstacle_avoidance_service_client(
              node_handle.serviceClient<ascend_msgs::ObstacleAvoidanceService>("obstacle_avoidance/path")) {}

    void initialize() override;
    void tick() override;
};

#endif
