//
// Created by simengangstad on 26.10.18.
//

#include "../../include/fluid/core/state.h"
#include "../../include/fluid/core/core.h"
#include <mavros_msgs/PositionTarget.h>
#include <utility>

fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    bool should_check_obstacle_avoidance_completion) : 

					Identifiable(identifier),
                    px4_mode(px4_mode),
					pose_subscriber_(node_handle_.subscribe("mavros/local_position/pose", 
                                     Core::message_queue_size, 
                                     &State::poseCallback, 
                                     this)),
                    twist_subscriber_(node_handle_.subscribe("mavros/local_position/twist", 
                                      Core::message_queue_size,
                                      &State::twistCallback,
                                      this)),
                    setpoint_publisher(node_handle_.advertise<mavros_msgs::PositionTarget>("fluid_fsm/setpoint", Core::message_queue_size)),
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

void fluid::State::obstacleAvoidanceCompletionCallback(const ascend_msgs::ObstacleAvoidanceCompletion& msg) {

    // Check whether the obstacle avoidance is returning completed on the current setpoint
    if (msg.setpoint.position.x != setpoint.position.x || 
        msg.setpoint.position.y != setpoint.position.y || 
        msg.setpoint.position.z != setpoint.position.z) {
        return;
    }

    if (!obstacle_avoidance_completed_ && msg.completed) {
        obstacle_avoidance_completed_ = true;
    }
}

void fluid::State::perform(std::function<bool(void)> shouldAbort) {

    ros::Rate rate(Core::refresh_rate);
    obstacle_avoidance_completed_ = false;

    while (ros::ok() && !hasFinishedExecution() && !shouldAbort()) {
        tick();

        setpoint_publisher.publish(setpoint);
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();

        if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state            
            break;
        }        
    }
}