#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

static geometry_msgs::msg::TransformStamped transform_stamped;

void poseCallback(const nav_msgs::msg::Odometry::SharedPtr &msg, rclcpp::Node node) {
    static tf2_ros::TransformBroadcaster broadcaster= tf2_ros::TransformBroadcaster(node);

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
    rclcpp::init(argc, argv);

    // transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("base_link_publisher");
    auto pose_subscriber = node->create_subscription<nav_msgs::msg::Odometry>("/mavros/global_position/local", 10, &poseCallback);

    rclcpp::spin(node);

    return 0;
};