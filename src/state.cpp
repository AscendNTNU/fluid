//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <utility>

#include "core.h"

fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    bool steady,
                    bool should_check_obstacle_avoidance_completion) : 

					identifier(identifier),
                    px4_mode(px4_mode),
                    steady_(steady),
					pose_subscriber_(node_handle_.subscribe("mavros/local_position/pose", 
                                     Core::message_queue_size, 
                                     &State::poseCallback, 
                                     this)),
                    twist_subscriber_(node_handle_.subscribe("mavros/local_position/twist", 
                                      Core::message_queue_size,
                                      &State::twistCallback,
                                      this)),
                    setpoint_publisher(node_handle_.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", Core::message_queue_size)),
                    should_check_obstacle_avoidance_completion_(should_check_obstacle_avoidance_completion) {}

void fluid::State::setCurrentPose(geometry_msgs::PoseStamped currentPose) {
    current_pose_ = currentPose;
}

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

std::vector<std::vector<float>> getSplineForPath() {

    // TODO: Fix service
    return {{0}, {0}, {0}};
}


void fluid::State::publishSetpoint() {
    setpoint_publisher.publish(setpoint);
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();

    while (ros::ok() && ((should_halt_if_steady && steady_) || !hasFinishedExecution()) && tick()) {
        this->tick();

        publishSetpoint();

        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();

        /*if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        } */       
    }
}