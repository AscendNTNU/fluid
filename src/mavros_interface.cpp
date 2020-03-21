/**
 * @file mavros_state_link.cpp
 * 
 * @brief Implementation of the Mavros Interface.
 */

#include "mavros_interface.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>

#include "type_mask.h"

MavrosInterface::MavrosInterface() {
    ros::NodeHandle node_handle;
    state_subscriber = node_handle.subscribe<mavros_msgs::State>("mavros/state", 1, &MavrosInterface::stateCallback, this);
    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
}

void MavrosInterface::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

mavros_msgs::State MavrosInterface::getCurrentState() const {
    return current_state;
}

void MavrosInterface::establishContactToPX4() const {
    ros::Rate rate(UPDATE_REFRESH_RATE);

    ROS_INFO("Attempting to establish contact with PX4...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}

bool MavrosInterface::attemptToSetState(const std::string& mode) const {
    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (getCurrentState().mode == mode) {
        return true;
    } else {
        ros::NodeHandle node_handle;
        ros::ServiceClient set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        return set_mode_client.call(set_mode);
    }
}
void MavrosInterface::requestArm(const bool& auto_arm) const {
    ros::Rate rate(UPDATE_REFRESH_RATE);

    // send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::IDLE;

    for (int i = UPDATE_REFRESH_RATE * 2; ros::ok() && i > 0; --i) {
        setpoint_publisher.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    // Arming
    ROS_INFO_STREAM("Attemping to arm...");

    if (!auto_arm) {
        ROS_INFO("Waiting for arm signal...");
    }

    ros::Time last_request = ros::Time::now();
    ros::NodeHandle node_handle;
    ros::ServiceClient arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;
    bool armed = false;
    double arm_request_interval = 0.5;

    while (ros::ok() && !armed) {
        // Send request to arm every interval specified
        if (ros::Time::now() - last_request > ros::Duration(arm_request_interval)) {
            if (!getCurrentState().armed) {
                if (auto_arm) {
                    if (arming_client.call(arm_command) && arm_command.response.success) {
                        armed = true;
                    }
                }
            } else {
                armed = true;
            }

            last_request = ros::Time::now();
        }

        setpoint_publisher.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}

void MavrosInterface::requestOffboard(const bool& auto_offboard) const {
    ros::Rate rate(UPDATE_REFRESH_RATE);
    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::IDLE;

    // Offboard
    ROS_INFO("Trying to set offboard...");

    if (!auto_offboard) {
        ROS_INFO("Waiting for offboard signal...");
    }

    bool set_offboard = false;

    while (ros::ok() && !set_offboard) {
        set_offboard = getCurrentState().mode == "OFFBOARD";

        if (auto_offboard) {
            set_offboard = attemptToSetState("OFFBOARD");
        }

        setpoint_publisher.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}

void MavrosInterface::setParam(const std::string& parameter, const float& value) const {
    ros::Rate rate(UPDATE_REFRESH_RATE);
    ros::NodeHandle node_handle;
    ros::ServiceClient param_set_service_client = node_handle.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    mavros_msgs::ParamSet param_set_service;

    param_set_service.request.param_id = parameter;
    param_set_service.request.value.real = value;

    bool failed_setting = false;

    while (!param_set_service_client.call(param_set_service) && ros::ok()) {
        if (!failed_setting) {
            ROS_FATAL_STREAM("Failed to set param " << parameter << " for PX4. Retrying...");
            failed_setting = true;
        }

        rate.sleep();
        ros::spinOnce();
    }
}
