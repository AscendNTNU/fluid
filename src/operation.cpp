/**
 * @file operation.cpp
 */

#include "operation.h"

#include <mavros_msgs/PositionTarget.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include "fluid.h"
#include "util.h"

Operation::Operation(const OperationIdentifier& identifier, const bool& steady, const bool& autoPublish)
                                        : identifier(identifier), steady(steady), autoPublish(autoPublish){
    pose_subscriber = node_handle.subscribe("mavros/global_position/local", 1, &Operation::poseCallback, this);
    twist_subscriber =
        node_handle.subscribe("mavros/local_position/velocity_local", 1, &Operation::twistCallback, this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    rate_int = (int) Fluid::getInstance().configuration.refresh_rate;
}


geometry_msgs::PoseStamped Operation::getCurrentPose() const { return current_pose; }

void Operation::poseCallback(const nav_msgs::Odometry::ConstPtr pose) {
    current_pose.pose = pose->pose.pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped Operation::getCurrentTwist() const { return current_twist; }


void Operation::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    previous_twist.twist.linear = current_twist.twist.linear;
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
    
    current_accel.linear.x = (current_twist.twist.linear.x - previous_twist.twist.linear.x);
    current_accel.linear.y = (current_twist.twist.linear.y - previous_twist.twist.linear.y);
    current_accel.linear.z = (current_twist.twist.linear.z - previous_twist.twist.linear.z);
}

geometry_msgs::Accel Operation::getCurrentAccel() const { return current_accel; }

geometry_msgs::Vector3 Operation::orientation_to_acceleration(geometry_msgs::Quaternion orientation)
{
    geometry_msgs::Vector3 accel;
    geometry_msgs::Vector3 angle = Util::quaternion_to_euler_angle(orientation);
    accel.x = tan(angle.y) *9.81;
    accel.y = -tan(angle.x) *9.81;
    accel.z = 0.0; //we actually don't know ...
    return accel;
}

float Operation::getCurrentYaw() const {
    geometry_msgs::Quaternion quaternion = current_pose.pose.orientation;
    geometry_msgs::Vector3 euler = Util::quaternion_to_euler_angle(quaternion);
    return euler.z;
}

void Operation::publishSetpoint() { 
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint_publisher.publish(setpoint); 
}

void Operation::perform(std::function<bool(void)> should_tick, bool should_halt_if_steady) {

    ros::Rate rate(rate_int);
    initialize();

    do {
        tick();
        if (autoPublish)
            publishSetpoint();
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.x = setpoint.position.x;
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.y = setpoint.position.y;
        Fluid::getInstance().getStatusPublisherPtr()->status.setpoint.z = setpoint.position.z;
        Fluid::getInstance().getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    } while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick());
}