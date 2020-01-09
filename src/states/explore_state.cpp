#include "explore_state.h"
#include "util.h"
#include <limits>

void ExploreState::initialize() {
    MoveState::initialize();

    retrieved_point_of_interest = false;
    std::string point_of_interest_topic;

    node_handle.param<std::string>("explore_point_of_interest_topic", point_of_interest_topic,
                                   "ai/explore_point_of_interest");
    point_of_interest_subscriber = node_handle.subscribe<geometry_msgs::Point>(
        point_of_interest_topic, 1, &ExploreState::pointOfInterestCallback, this);

    original_path = path;
}

void ExploreState::pointOfInterestCallback(const geometry_msgs::PointConstPtr& point) {
    point_of_interest = *point;
    retrieved_point_of_interest = true;
}

void ExploreState::tick() {

    ascend_msgs::ObstacleAvoidanceService service;
    service.request.path = original_path;

    if (obstacle_avoidance_service_client.call(service)) {
        // Check if the path is different from the current path

        bool different_path = path.size() != service.response.new_path.size();
        double closest_distance = std::numeric_limits<double>::max();
        unsigned int closest_point_index = -1;

        // Find the point we are closest to in the path given from OA and set that as starting point for the
        // iterator if the paths are different
        if (path.size() == service.response.new_path.size()) {
            for (int i = 0; i < path.size(); i++) {
                double distance = Util::distanceBetween(path[i], service.response.new_path[i]);

                if (distance >= 0.01) {
                    different_path = true;
                }

                if (distance <= closest_distance) {
                    closest_distance = distance;
                    closest_point_index = i;
                }
            }
        }

        if (different_path) {
            if (closest_point_index != -1) {
                path = service.response.new_path;
                current_destination_point_iterator = path.begin() + closest_point_index;
            }
            else {
                ROS_FATAL_STREAM(
                    "Could not find a closest point even though the paths were different, did the obstacle avoidance "
                    "service return a finite path of infinite numbers? Will not change path.");
            }
        }
    } else {
        ROS_FATAL_STREAM("Failed to call obstacle avoidance service!");
    }

    MoveState::tick();
    
    if (retrieved_point_of_interest) {
        double dx = point_of_interest.x - getCurrentPose().pose.position.x;
        double dy = point_of_interest.y - getCurrentPose().pose.position.y;

        setpoint.yaw = std::atan2(dy, dx);
    }
}
