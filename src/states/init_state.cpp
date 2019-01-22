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



bool fluid::InitState::hasFinishedExecution() {
    return armed;
}

void fluid::InitState::tick() {
    // Not implemented as all logic happens inside perform for the init state has we have to arm and set offboard mode
}

void fluid::InitState::perform(std::function<bool (void)> shouldAbort) {

    ros::Rate rate(refresh_rate_);

    fluid::MavrosStateSetter state_setter(node_handle_p, 1000, 1.0/static_cast<double>(refresh_rate_), "OFFBOARD");
    ros::ServiceClient arming_client = node_handle_p->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;

    ROS_INFO_STREAM("Attempting to arm...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !state_setter.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    position_target.position.x = 0;
    position_target.position.y = 0;
    position_target.position.z = 0;

    // TODO: Evaluate numbers here
    for (int i = 100; ros::ok() && i > 0; --i) {
        position_target_publisher_p->publish(position_target);
        ros::spinOnce();
        rate.sleep();
    }


    // Attempt to set offboard mode. Once that has completed, arm the drone.
    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !hasFinishedExecution()) {

        state_setter.attemptToSetState([&](bool completed) {
            if(completed &&
               !state_setter.getCurrentState().armed &&
               (ros::Time::now() - last_request > ros::Duration(1.0))) {

                if(arming_client.call(arm_command) && arm_command.response.success){
                    ROS_INFO("Drone armed");
                    armed = true;
                }

                last_request = ros::Time::now();
            }
        });

        position_target_publisher_p->publish(position_target);

        ros::spinOnce();
        rate.sleep();
    }
}
