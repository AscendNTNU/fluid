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
                    setpoint_publisher(node_handle_.advertise<ascend_msgs::ObstacleAvoidanceSetpoint>("fluid/setpoint", Core::message_queue_size)),
                    obstacle_avoidance_completion_subscriber_(node_handle_.subscribe("obstacle_avoidance/completion", 
                                                              Core::message_queue_size, 
                                                              &State::obstacleAvoidanceCompletionCallback, 
                                                              this)),
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

void fluid::State::obstacleAvoidanceCompletionCallback(const ascend_msgs::ObstacleAvoidanceCompletion& msg) {
  
    // Check whether the obstacle avoidance is returning completed on the current setpoint
    if (abs(msg.setpoint.position.x - setpoint.position.x) >= 0.01 || 
        abs(msg.setpoint.position.y - setpoint.position.y) >= 0.01 || 
        abs(msg.setpoint.position.z - setpoint.position.z) >= 0.01 ||
        abs(msg.setpoint.yaw - setpoint.yaw) >= 0.01) {
        return;
    }

    if (!obstacle_avoidance_completed_ && msg.completed) {
        obstacle_avoidance_completed_ = true;
    }
}

void fluid::State::publishSetpoint() {
    ascend_msgs::ObstacleAvoidanceSetpoint obstacle_avoidance_setpoint;

    obstacle_avoidance_setpoint.setpoint = setpoint;
    // We pass the current state so the obstaclee avoidance filter can adjust the behaviour for 
    // the given state.
    obstacle_avoidance_setpoint.state = identifier;
    
    setpoint_publisher.publish(obstacle_avoidance_setpoint);
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);
    obstacle_avoidance_completed_ = false;

    initialize();

    while (ros::ok() && ((should_halt_if_steady && steady_) || !hasFinishedExecution()) && !tick()) {
        this->tick();

        publishSetpoint();
        fluid::Core::getStatusPublisherPtr()->status.setpoint = setpoint;
        fluid::Core::getStatusPublisherPtr()->publish();

        ros::spinOnce();
        rate.sleep();

        if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        }        
    }
}