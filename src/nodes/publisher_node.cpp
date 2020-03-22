/**
 * @file publisher_node.cpp
 *
 * @brief This node is a safety measure in case the main thread in the FSM hangs some and
 *        we therefore don't will publish setpoints to PX4 regularly. It grabs the last
 *        published setpoint from the FSM and publishes that.
 *        It also publishes the base link transform
 */

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <string>

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

    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Initiailzing set point publisher.");

    int refresh_rate = 20;
    ros::NodeHandle node_handle;

    if (!node_handle.getParam("refresh_rate", refresh_rate)) {
        ROS_FATAL_STREAM("Refresh rate parameter not specified, shutting down...");
        ros::shutdown();
        return 1;
    }

    ros::Subscriber subscriber = node_handle.subscribe("fluid/setpoint", 1, subscriptionCallback);
    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
    ros::Subscriber pose_subscriber = node_handle.subscribe("/mavros/local_position/pose", 1, &poseCallback);
    ros::Rate rate(refresh_rate);

    while (ros::ok()) {
        setpoint.header.stamp = ros::Time::now();

        if (position_is_set) {
            publisher.publish(setpoint);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
