//
// This node is a safety measure in case the main thread in the FSM is blocked and
// we therefore don't will publish setpoints to PX4 regularly. It grabs the last
// published setpoint from the FSM and publishes that.
//
// It also publishes the base link transform

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <string>

#include "core.h"

mavros_msgs::PositionTarget setpoint;
bool position_is_set = false;

void subscriptionCallback(const mavros_msgs::PositionTarget::ConstPtr& pt) {
    setpoint = *pt;
    position_is_set = true;
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStampedNonOriented;

    transformStampedNonOriented.header.stamp = ros::Time::now();
    transformStampedNonOriented.header.frame_id = "odom";
    transformStampedNonOriented.child_frame_id = "base_link";
    transformStampedNonOriented.transform.translation.x = msg->pose.position.x;
    transformStampedNonOriented.transform.translation.y = msg->pose.position.y;
    transformStampedNonOriented.transform.translation.z = msg->pose.position.z;

    transformStampedNonOriented.transform.rotation.x = 0;
    transformStampedNonOriented.transform.rotation.y = 0;
    transformStampedNonOriented.transform.rotation.z = 0;
    transformStampedNonOriented.transform.rotation.w = 1.0;

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "base_link_oriented";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    transformStamped.transform.rotation.x = msg->pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.orientation.w;

    broadcaster.sendTransform(transformStampedNonOriented);
    broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_setpoint_publisher");

    ROS_INFO("Initiailzing set point publisher.");

    int refresh_rate = Core::refresh_rate;
    ros::NodeHandle node_handle;

    node_handle.getParam("refresh_rate", refresh_rate);

    std::string subscription_topic = std::string(argv[1]);
    std::string publishing_topic = std::string(argv[2]);

    ros::Subscriber subscriber = node_handle.subscribe(subscription_topic, 1, subscriptionCallback);
    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>(publishing_topic, 1);
    ros::Subscriber pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1, &poseCallback);
    ros::Publisher setpoint_visualizer_publisher = node_handle.advertise<visualization_msgs::Marker>("/fluid/setpoint_visualiztion", 10);

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.id = 9999;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
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

    ros::Rate rate(refresh_rate);

    while (ros::ok()) {
        setpoint.header.stamp = ros::Time::now();

        if (position_is_set) {
            marker.pose.position.x = setpoint.position.x;
            marker.pose.position.y = setpoint.position.y;
            marker.pose.position.z = setpoint.position.z;

            setpoint_visualizer_publisher.publish(marker);
            publisher.publish(setpoint);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
