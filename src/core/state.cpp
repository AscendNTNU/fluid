//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"
#include <ros/ros.h>

ros::NodeHandle fluid::State::node_handle_;
fluid::MavrosPosePublisher fluid::State::publisher(node_handle_, 1000);

void fluid::State::perform() {

    ros::Rate rate(refresh_rate_);

    while(ros::ok() && !hasFinishedExecution()) {
        publisher.publish(pose);

        tick();

        ros::spinOnce();
        rate.sleep();
    }
}