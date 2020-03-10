/**
 * @file mavros_state_link.cpp
 * 
 * @brief Implementation of the Mavros Interface.
 */
#include "mavros_interface.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include "type_mask.h"

MavrosInterface::MavrosInterface() : state_subscriber(node_handle.subscribe<mavros_msgs::State>("mavros/state", 1, &MavrosInterface::stateCallback, this)),
                                     set_mode_client(node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")),
                                     setpoint_publisher(node_handle.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10)) {}

void MavrosInterface::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

mavros_msgs::State MavrosInterface::getCurrentState() {
    return current_state;
}

void MavrosInterface::establishContactToPX4() {
    ros::Rate rate(UPDATE_REFRESH_RATE);

    ROS_INFO("Attempting to establish contact with PX4...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}

void MavrosInterface::attemptToSetState(std::string mode, std::function<void(bool)> completion_handler) {
    // The state on the Pixhawk is equal to the state we wan't to set, so we just return
    if (getCurrentState().mode == mode) {
        completion_handler(true);
    } else {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (ros::Time::now() - last_request_time > ros::Duration(1.0 / static_cast<float>(UPDATE_REFRESH_RATE))) {
            if (set_mode_client.call(set_mode)) {
                completion_handler(set_mode.response.mode_sent);
            }

            last_request_time = ros::Time::now();
        }
    }
}
void MavrosInterface::requestArm(const bool auto_arm) {
    ros::Rate rate(UPDATE_REFRESH_RATE);

    // send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Idle;

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

void MavrosInterface::requestOffboard(const bool auto_offboard) {
    ros::Rate rate(UPDATE_REFRESH_RATE);
    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Idle;

    // Offboard
    ROS_INFO("Trying to set offboard...");

    if (!auto_offboard) {
        ROS_INFO("Waiting for offboard signal...");
    }

    bool set_offboard = false;

    while (ros::ok() && !set_offboard) {
        set_offboard = getCurrentState().mode == "OFFBOARD";

        if (auto_offboard) {
            attemptToSetState("OFFBOARD", [&](bool completed) {
                set_offboard = completed;
            });
        }

        setpoint_publisher.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}
