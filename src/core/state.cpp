//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"
#include "../../include/mavros/mavros_setpoint_msg_defines.h"
#include "../../include/core/core.h"
#include <utility>

fluid::State::State(fluid::StateIdentifier identifier,
                    std::string px4_mode,
                    std::string pose_subscription_topic,
                    std::shared_ptr<fluid::PosePublisher> position_target_publisher_p) : 

					Identifiable(identifier),
                    px4_mode(px4_mode),
					pose_subscriber_(node_handle_.subscribe(pose_subscription_topic, 
                                     Core::message_queue_size, 
                                     &State::poseCallback, 
                                     this)),
                    position_target_publisher_p(std::move(position_target_publisher_p)) {}

geometry_msgs::PoseStamped fluid::State::getCurrentPose() {
	return current_pose_;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose_.pose = pose->pose;
    current_pose_.header = pose->header;
}

void fluid::State::perform(std::function<bool(void)> shouldAbort) {

    ros::Rate rate(Core::refresh_rate);

    while (ros::ok() && !hasFinishedExecution() && !shouldAbort()) {
        tick();

        position_target_publisher_p->publish(position_target);
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();
    }
}