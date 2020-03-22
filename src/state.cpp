/**
 * @file state.cpp
 */

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <utility>

#include "core.h"
#include "util.h"

State::State(const StateIdentifier& identifier, const bool& steady) : identifier(identifier), steady(steady) {
    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &State::poseCallback, this);
    twist_subscriber = node_handle.subscribe("mavros/local_position/velocity_local", 1, &State::twistCallback, this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", 10);
}

geometry_msgs::PoseStamped State::getCurrentPose() const { return current_pose; }

void State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped State::getCurrentTwist() const { return current_twist; }

void State::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}

float State::getCurrentYaw() const {
    geometry_msgs::Quaternion quaternion = current_pose.pose.orientation;
    tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
    // it to zero.
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    return yaw;
}

void State::publishSetpoint() { setpoint_publisher.publish(setpoint); }

void State::perform(std::function<bool(void)> should_tick, bool should_halt_if_steady) {
    ros::Rate rate(Core::refresh_rate);

    initialize();

    do {
        tick();
        publishSetpoint();

        Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    } while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick());

    finalize();
}