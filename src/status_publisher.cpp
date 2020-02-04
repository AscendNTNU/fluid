#include "status_publisher.h"
#include "core.h"

#include <nav_msgs/Path.h>

StatusPublisher::StatusPublisher() {
    status.armed = 0;
    status.linked_with_px4 = 0;
    status.px4_mode = "none";
    status.current_operation = "none";
    status.current_state = "none";
    status.path = {};

    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &StatusPublisher::poseCallback, this);
    status_publisher = node_handle.advertise<ascend_msgs::FluidStatus>("fluid/status", 1);
    trace_publisher = node_handle.advertise<nav_msgs::Path>("fluid/trace", 1);
    path_publisher = node_handle.advertise<nav_msgs::Path>("fluid/path", 1);
}

void StatusPublisher::poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr) {
    if (trace_path.poses.size() > 300) {
        trace_path.poses.erase(trace_path.poses.begin());
    }

    trace_path.poses.push_back(*pose_ptr);
}

void StatusPublisher::publish() {
    status_publisher.publish(status);
    trace_path.header.stamp = ros::Time::now();
    trace_path.header.frame_id = "map";
    trace_publisher.publish(trace_path);

    path.poses.clear();

    for (auto point : status.path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position = point;
        path.poses.push_back(pose_stamped);
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path_publisher.publish(path);
}
