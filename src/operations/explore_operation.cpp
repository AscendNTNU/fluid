/**
 * @file explore_operation.cpp
 */

#include "explore_operation.h"
#include "mavros_interface.h"

#include <limits>
#include <std_srvs/Trigger.h>

#include "util.h"

ExploreOperation::ExploreOperation(const std::vector<geometry_msgs::Point>& path, const geometry_msgs::Point& point_of_interest)
    : MoveOperation(OperationIdentifier::EXPLORE, path, 1, 0.5, 1, 15),
      obstacle_avoidance_path_publisher(node_handle.advertise<ascend_msgs::Path>("/obstacle_avoidance/path", 10)),
      obstacle_avoidance_path_subscriber(
          node_handle.subscribe("/obstacle_avoidance/corrected_path", 10, &ExploreOperation::pathCallback, this)),
      point_of_interest(point_of_interest) {}

void ExploreOperation::initialize() {
    //Face straight ahead
    if (path.size() == 0) {
        path.push_back(getCurrentPose().pose.position);
    }

    MoveOperation::initialize();

    MavrosInterface mavros_interface;
    mavros_interface.setParam("WPNAV_ACCEL", 50);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max acceleration to: " << 50/100.0 << " m/s2.");

    ros::ServiceClient fh_extend = node_handle.serviceClient<std_srvs::Trigger>("/facehugger/moveforward");
    std_srvs::Trigger fh_extend_handle;
    if (fh_extend.call(fh_extend_handle))   ROS_INFO("Facehugger extend called");
    else                                    ROS_INFO("Facehugger extend-call failed");

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

void ExploreOperation::pathCallback(ascend_msgs::Path corrected_path) {
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
                path = std::vector<geometry_msgs::Point>(corrected_path.points.begin() + closest_point_index,
                                                         corrected_path.points.end());
                current_setpoint_iterator = path.begin();
                update_setpoint = true;

            } else {
                ROS_FATAL_STREAM(ros::this_node::getName().c_str()
                                 << ": Could not find a closest point even though the paths were different, "
                                 << "did the obstacle avoidance service return a finite path of infinite "
                                 << "numbers? Will not change path.");
            }
        }
    }
}

void ExploreOperation::tick() {
    MoveOperation::tick();

    double dx = point_of_interest.x - getCurrentPose().pose.position.x;
    double dy = point_of_interest.y - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);
   
    ascend_msgs::Path path_msg;
    path_msg.points = dense_path;
    obstacle_avoidance_path_publisher.publish(path_msg);
}
