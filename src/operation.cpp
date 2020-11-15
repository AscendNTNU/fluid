/**
 * @file operation.cpp
 */

#include "operation.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <utility>

#include "fluid.h"
#include "util.h"

Operation::Operation(const OperationIdentifier& identifier, const bool& steady)
    : identifier(identifier), steady(steady) {
    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &Operation::poseCallback, this);
    twist_subscriber =
        node_handle.subscribe("mavros/local_position/velocity_local", 1, &Operation::twistCallback, this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", 10);
    setpoint.coordinate_frame = 1;
}

geometry_msgs::PoseStamped Operation::getCurrentPose() const { return current_pose; }

void Operation::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped Operation::getCurrentTwist() const { return current_twist; }

void Operation::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}

float Operation::getCurrentYaw() const {
    geometry_msgs::Quaternion quaternion = current_pose.pose.orientation;
    tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
    // it to zero.
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    return yaw;
}

void Operation::publishSetpoint() { setpoint_publisher.publish(setpoint); }

void Operation::perform(std::function<bool(void)> should_tick, bool should_halt_if_steady) {
    ros::Rate rate(Fluid::getInstance().configuration.refresh_rate);

    initialize();

    do {
        tick();
        publishSetpoint();
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.x = setpoint.position.x;
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.y = setpoint.position.y;
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.z = setpoint.position.z;
        Fluid::getInstance().getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    } while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick());
}