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

}

void fluid::InitState::tick() {

}


void fluid::InitState::perform() {

    /*ros::Rate rate(20); // Arbitrary number, must be higher than 2 Hz

    geometry_msgs::PoseStamped pose;

    // Run until we connect with mavros, init specific code
    {
        while (ros::ok() && !state_setter.state_subscriber.current_state.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        //send a few setpoints before starting. This is because the stream has to be set ut before we change modes within px4
        for (int i = 100; ros::ok() && i > 0; --i) {
            publisher.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }


    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }*/
}
