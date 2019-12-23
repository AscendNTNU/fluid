//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include <utility>

#include "core.h"
#include "util.h"

State::State(const StateIdentifier& identifier, const PX4StateIdentifier& px4_mode, const bool& steady,
             const bool& should_check_obstalce_avoidance_completion)
    : identifier(identifier),
      px4_mode(px4_mode),
      steady(steady),
      should_check_obstacle_avoidance_completion(should_check_obstacle_avoidance_completion) {
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

void State::publishSetpoint() { setpoint_publisher.publish(setpoint); }

void State::perform(std::function<bool(void)> should_tick, bool should_halt_if_steady) {
    ros::Rate rate(Core::refresh_rate);

    initialize();

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick()) {
        tick();
        publishSetpoint();

        Core::getStatusPublisherPtr()->status.path = path;
        Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    }
}