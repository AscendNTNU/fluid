//
//  Created by Simen Gangstad on 15/10/2018.
//


#include "../../include/states/init_state.h"

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../include/core/core.h"
#include "../../include/mavros/mavros_state_setter.h"

bool fluid::InitState::hasFinishedExecution() {
    return armed;
}

void fluid::InitState::tick() {
    // Not implemented as all logic happens inside perform for the init state has we have to arm and set offboard mode
}

void fluid::InitState::perform(std::function<bool (void)> shouldAbort) {

    ros::Rate rate(Core::refresh_rate);
    ros::NodeHandle node_handle_;


    // Establishing contact through mavros with Pixhawk.

    fluid::MavrosStateSetter state_setter(Core::message_queue_size, 
    									  1.0/static_cast<double>(Core::refresh_rate), 
    									  "OFFBOARD");

    ROS_INFO("Attempting to establish contact with PX4...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !state_setter.getCurrentState().connected) {
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    fluid::Core::getStatusPublisherPtr()->status.linked_with_px4 = 1;

    //send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    position_target.position.x = 0;
    position_target.position.y = 0;
    position_target.position.z = 0;

    for (int i = Core::refresh_rate*3; ros::ok() && i > 0; --i) {
        position_target_publisher_p->publish(position_target);
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
    }



    // Offboard

    ROS_INFO("Attempting to set offboard...");

    bool set_offboard = false;

    while(ros::ok() && !hasFinishedExecution() && !set_offboard) {

        state_setter.attemptToSetState([&](bool completed) {

            set_offboard = completed;

            if (completed) {
                fluid::Core::getStatusPublisherPtr()->status.px4_mode = "offboard";
            }
        });

        fluid::Core::getStatusPublisherPtr()->publish();
        position_target_publisher_p->publish(position_target);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    ROS_INFO_STREAM("Attemping to arm... Auto arm: " << fluid::Core::auto_arm);

    if (fluid::Core::auto_arm) {
        ROS_INFO("Waiting for arm signal...");
    }

    // Arming

    ros::Time last_request = ros::Time::now();
    ros::ServiceClient arming_client = node_handle_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;
    bool armed = false;
    double arm_request_interval = 0.5;

    while (ros::ok() && !hasFinishedExecution() && !armed) {

        // Send request to arm every interval specified
        if (ros::Time::now() - last_request > ros::Duration(arm_request_interval)) {

            if (!state_setter.getCurrentState().armed) {
                if (fluid::Core::auto_arm) {
                    if(arming_client.call(arm_command) && arm_command.response.success) {
                        fluid::Core::getStatusPublisherPtr()->status.armed = 1;
                        armed = true;
                    }
                }
            }
            else {
                fluid::Core::getStatusPublisherPtr()->status.armed = 1;
                armed = true;
            }

            last_request = ros::Time::now();
        }

        fluid::Core::getStatusPublisherPtr()->publish();
        position_target_publisher_p->publish(position_target);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");
}
