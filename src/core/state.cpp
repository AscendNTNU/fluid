//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"
#include "../../include/core/fluid_fsm.h"

geometry_msgs::PoseStamped fluid::State::getCurrentPose() {
	return current_pose_;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose_.pose = pose->pose;
    current_pose_.header = pose->header;
}

void fluid::State::perform(std::function<bool(void)> shouldAbort) {

    ros::Rate rate(refresh_rate_);

    while (ros::ok() && !hasFinishedExecution() && !shouldAbort()) {
        tick();

        position_target_publisher_p->publish(position_target);
        status_publisher.publish();

        ros::spinOnce();
        rate.sleep();
    }
}