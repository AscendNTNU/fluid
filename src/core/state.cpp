//
// Created by simengangstad on 26.10.18.
//

#include "../../include/core/state.h"
#include "../../include/core/core.h"
#include <utility>


fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    std::string pose_subscription_topic,
                    std::string twist_subscription_topic,
                    std::shared_ptr<fluid::PosePublisher> position_target_publisher_p,
                    bool should_check_obstacle_avoidance_completion) : 

					Identifiable(identifier),
                    px4_mode(px4_mode),
					pose_subscriber_(node_handle_.subscribe(pose_subscription_topic, 
                                     Core::message_queue_size, 
                                     &State::poseCallback, 
                                     this)),
                    twist_subscriber_(node_handle_.subscribe(twist_subscription_topic, 
                                      Core::message_queue_size,
                                      &State::twistCallback,
                                      this)),
                    position_target_publisher_p(std::move(position_target_publisher_p)),
                    obstacle_avoidance_completion_subscriber_(node_handle_.subscribe("obstacle_avoidance/completion", 
                                                              Core::message_queue_size, 
                                                              &State::obstacleAvoidanceCompletionCallback, 
                                                              this)),
                    should_check_obstacle_avoidance_completion_(should_check_obstacle_avoidance_completion) {}

geometry_msgs::PoseStamped fluid::State::getCurrentPose() {
	return current_pose_;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose_.pose = pose->pose;
    current_pose_.header = pose->header;
}

geometry_msgs::TwistStamped fluid::State::getCurrentTwist() {
    return current_twist_;
}

void fluid::State::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist_.twist = twist->twist;
    current_twist_.header = twist->header;
}

void fluid::State::obstacleAvoidanceCompletionCallback(const std_msgs::Bool::ConstPtr& completed) {
    if (!obstacle_avoidance_completed_ && completed->data) {
        obstacle_avoidance_completed_ = true;
    }
}

void fluid::State::perform(std::function<bool(void)> shouldAbort) {

    ros::Rate rate(Core::refresh_rate);
    obstacle_avoidance_completed_ = false;

    while (ros::ok() && !hasFinishedExecution() && !shouldAbort()) {
        tick();

        position_target_publisher_p->publish(position_target);
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();

        if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            
            ROS_FATAL("Obstacle avoidance completed");
            break;
        }        
    }
}