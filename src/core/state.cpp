//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"

void fluid::State::perform() {

    ros::Rate rate(refresh_rate_);

    while(ros::ok() && !hasFinishedExecution()) {
        pose_publisher_p->publish(pose);

        tick();

        ros::spinOnce();
        rate.sleep();
    }
}