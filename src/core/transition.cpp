//
// Created by simengangstad on 04.10.18.
//

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <ros/ros.h>

#include "../../include/core/transition.h"
#include <algorithm>

/*
mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}*/


void fluid::Transition::perform() {

    TransitionErrorCode transition_error_code = no_error;

    // Copy pose to the new state
    // TODO: Is this really copied, or referenced?
    end_state_p->pose = start_state_p->pose;

    // TODO: Communicate with PX4 and get errors (if any) and set them in the error code

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arming_command;
    arming_command.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arming_command) && arming_command.response.success) {
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();
            }
        }

        local_position_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    const std::type_info& start_state_type_info = typeid(start_state_p.get());
    const std::type_info& end_state_type_info = typeid(end_state_p.get());

    TransitionError transition_error = {transition_error_code,
                                        start_state_type_info.name(),
                                        end_state_type_info.name()};

    if (auto transition_delegate = transition_delegate_p.lock()) {
        transition_delegate->completed(transition_error);
    }
}
