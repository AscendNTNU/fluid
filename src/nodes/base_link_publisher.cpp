#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/msg/transform_broadcaster.hpp>
#include <nav_msgs/Odometry.hpp>
#include <chrono>

static geometry_msgs::TransformStamped transform_stamped;

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static tf2_ros::TransformBroadcaster broadcaster;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    transform_stamped.header.stamp = now;
    transform_stamped.header.frame_id = msg->header.frame_id;
    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.pose.position.z;

    transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;

    broadcaster.sendTransform(transform_stamped);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, "base_link_tf_publisher");

    // transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    auto node = std::make_shared<typename _Tp>(_Args &&__args...)
    rclcpp::Subscription<nav_msgs::Odometry::ConstPtr>::SharedPtr pose_subscriber = subscribe("/mavros/global_position/local", 10, &poseCallback);

    ros::spin();

    return 0;
};