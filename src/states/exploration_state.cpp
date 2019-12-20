#include "exploration_state.h"


void fluid::ExplorationState::initialize() {
    MoveState::initialize();

    retrieved_point_of_interest = false;
    std::string point_of_interest_topic;

    node_handle.param<std::string>("exploration_point_of_interest_topic", point_of_interest_topic, "ai/exploration_point_of_interest");
    point_of_interest_subscriber = node_handle.subscribe<geometry_msgs::Point>(point_of_interest_topic, 1, &ExplorationState::pointOfInterestCallback, this);
}

void fluid::ExplorationState::pointOfInterestCallback(const geometry_msgs::PointConstPtr& point) {
   point_of_interest = *point; 
   retrieved_point_of_interest = true;
}

void fluid::ExplorationState::tick() {
    MoveState::tick();

    if (retrieved_point_of_interest) {
        double dx = point_of_interest.x - getCurrentPose().pose.position.x; 
        double dy = point_of_interest.y - getCurrentPose().pose.position.y; 

        setpoint.yaw = std::atan2(dy, dx);
    }
}
