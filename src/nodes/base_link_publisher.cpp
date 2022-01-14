#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <nav_msgs/Odometry.hpp>

static geometry_msgs::TransformStamped transform_stamped;

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static tf2_ros::TransformBroadcaster broadcaster;

    transform_stamped.header.stamp = ros::Time::now();
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
    ros::init(argc, argv, "base_link_tf_publisher");

    // transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";

    ros::NodeHandle node;
    ros::Subscriber pose_subscriber = node.subscribe("/mavros/global_position/local", 10, &poseCallback);

    ros::spin();

    return 0;
};