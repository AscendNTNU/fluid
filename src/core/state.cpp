//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"

void fluid::State::perform(std::function<bool (void)> shouldAbort) {

    ros::Rate rate(refresh_rate_);

    while(ros::ok() && !hasFinishedExecution() && !shouldAbort()) {
        tick();

        position_target_publisher_p->publish(position_target);
        
        ros::spinOnce();
        rate.sleep();
    }
}