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

void fluid::InitState::perform() {

    ros::Rate rate(refresh_rate_);

    ros::NodeHandle node_handle;
    fluid::MavrosStateSetter state_setter(node_handle, 1000, 2, "OFFBOARD");
    ros::ServiceClient arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;


    // Run until we achieve a connection with mavros
    while (ros::ok() && !state_setter.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }


    //send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    // TODO: Evaluate numbers here
    for (int i = 100; ros::ok() && i > 0; --i) {
        pose_publisher_p->publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    // Attempt to set offboard mode. Once that has completed, arm the drone.
    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !hasFinishedExecution()) {

        state_setter.attemptToSetState([&](bool completed) {
            if(completed &&
               !state_setter.getCurrentState().armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))) {

                if(arming_client.call(arm_command) && arm_command.response.success){
                    ROS_INFO("FLUID FSM - Drone armed");
                    armed = true;
                }

                last_request = ros::Time::now();
            }
        });

        pose_publisher_p->publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
}
