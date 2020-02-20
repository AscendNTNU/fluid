#include "explore_state.h"
#include <limits>
#include "util.h"

void ExploreState::initialize() {
    MoveState::initialize();

    retrieved_point_of_interest = false;
    std::string point_of_interest_topic;

    node_handle.param<std::string>("explore_point_of_interest_topic", point_of_interest_topic,
                                   "ai/explore_point_of_interest");
    point_of_interest_subscriber = node_handle.subscribe<geometry_msgs::Point>(
        point_of_interest_topic, 1, &ExploreState::pointOfInterestCallback, this);

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

void ExploreState::pointOfInterestCallback(const geometry_msgs::PointConstPtr& point) {
    point_of_interest = *point;
    retrieved_point_of_interest = true;
}

void ExploreState::pathCallback(ascend_msgs::Path corrected_path) {
    //std::reverse(corrected_path.points.begin(), corrected_path.points.end());

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
                current_destination_point_iterator = path.begin();
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

    if (retrieved_point_of_interest) {
        double dx = point_of_interest.x - getCurrentPose().pose.position.x;
        double dy = point_of_interest.y - getCurrentPose().pose.position.y;

        setpoint.yaw = std::atan2(dy, dx);
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.id = 9999;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_destination_point_iterator->x;
    marker.pose.position.y = current_destination_point_iterator->y;
    marker.pose.position.z = current_destination_point_iterator->z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    current_setpoint_visualization_publisher.publish(marker);

    ascend_msgs::Path path_msg;

    path_msg.points = dense_path;
    obstacle_avoidance_path_publisher.publish(path_msg);
}

void ExploreState::finalize() { original_path_set = false; }
