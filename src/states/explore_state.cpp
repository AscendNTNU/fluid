/**
 * @file explore_state.cpp
 */

#include "explore_state.h"
#include <limits>
#include "util.h"

ExploreState::ExploreState() : MoveState(StateIdentifier::Explore, 0.3, 0.3, 0.3),
                               obstacle_avoidance_path_publisher(node_handle.advertise<ascend_msgs::Path>("/obstacle_avoidance/path", 10)),
                               obstacle_avoidance_path_subscriber(node_handle.subscribe("/obstacle_avoidance/corrected_path", 10, &ExploreState::pathCallback, this)) {}

void ExploreState::initialize() {
    MoveState::initialize();

    original_path = path;
    original_path_set = true;

    dense_path.clear();

    if (original_path.size() == 1) {
        dense_path.insert(dense_path.begin(), getCurrentPose().pose.position);
    }

    for (int i = 1; i < original_path.size(); i++) {
        std::vector<geometry_msgs::Point> filler_points =
            Util::createPath(original_path[i - 1], original_path[i], path_density);
        dense_path.insert(dense_path.end(), begin(filler_points), end(filler_points));
    }
}

void ExploreState::pathCallback(ascend_msgs::Path corrected_path) {
    if (original_path_set) {
        // Check if the path is different from the current path

        bool different_path = path.size() != corrected_path.points.size();
        unsigned int closest_point_index = -1;

        // Find the point we are closest to in the path given from OA and set that as starting point for the
        // iterator if the paths are different
        if (path.size() == corrected_path.points.size()) {
            for (int i = 0; i < path.size(); i++) {
                double distance = Util::distanceBetween(path[i], corrected_path.points[i]);

                if (distance >= 0.01) {
                    different_path = true;
                    break;
                }
            }
        }

        if (different_path) {
            double closest_distance = std::numeric_limits<double>::max();

            for (int i = 0; i < corrected_path.points.size(); i++) {
                double distance = Util::distanceBetween(getCurrentPose().pose.position, corrected_path.points[i]);

                if (distance <= closest_distance) {
                    closest_distance = distance;
                    closest_point_index = i;
                }
            }

            if (closest_point_index != -1) {
                path = std::vector<geometry_msgs::Point>(corrected_path.points.begin() + closest_point_index, corrected_path.points.end());
                current_setpoint_iterator = path.begin();
                update_setpoint = true;

            } else {
                ROS_FATAL_STREAM(
                    "Could not find a closest point even though the paths were different, did the obstacle avoidance "
                    "service return a finite path of infinite numbers? Will not change path.");
            }
        }
    }
}

void ExploreState::tick() {
    MoveState::tick();

    ascend_msgs::Path path_msg;
    path_msg.points = dense_path;
    obstacle_avoidance_path_publisher.publish(path_msg);
}

void ExploreState::finalize() { original_path_set = false; }
