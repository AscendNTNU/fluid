#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

static geometry_msgs::TransformStamped transform_stamped;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    static tf2_ros::TransformBroadcaster broadcaster;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.transform.translation.x = msg->pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.position.z;

    transform_stamped.transform.rotation.x = msg->pose.orientation.x;
    transform_stamped.transform.rotation.y = msg->pose.orientation.y;
    transform_stamped.transform.rotation.z = msg->pose.orientation.z;
    transform_stamped.transform.rotation.w = msg->pose.orientation.w;

    broadcaster.sendTransform(transform_stamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_link_tf_publisher");

    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_linke";

    ros::NodeHandle node;
    ros::Subscriber pose_subscriber = node.subscribe("/mavros/local_position/pose", 10, &poseCallback);

    ros::spin();

    return 0;
};