//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>

#include <utility>

#include "util.h"
#include "core.h"

fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    bool steady, 
                    bool should_check_obstalce_avoidance_completion) : 

					identifier(identifier),
                    px4_mode(px4_mode),
                    steady(steady), 
                    should_check_obstacle_avoidance_completion(should_check_obstacle_avoidance_completion) {

    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 
                                            Core::message_queue_size, 
                                            &State::poseCallback, 
                                            this);
    twist_subscriber = node_handle.subscribe("mavros/local_position/twist", 
                                             Core::message_queue_size, 
                                             &State::poseCallback, 
                                             this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", Core::message_queue_size);

    path_optimizer_client = node_handle.serviceClient<ascend_msgs::PathOptimizerService>("/control/path_optimizer");
}

geometry_msgs::PoseStamped fluid::State::getCurrentPose() const {
	return current_pose;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped fluid::State::getCurrentTwist() const {
    return current_twist;
}

void fluid::State::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}

void fluid::State::publishSetpoint() {
    setpoint_publisher.publish(setpoint);
}

fluid::ControllerType fluid::State::getPreferredController() const {
    return ControllerType::Passthrough;
}

std::vector<ascend_msgs::Spline> fluid::State::getSplineForPath(const std::vector<geometry_msgs::Point>& path) {
    // TODO: Temp, have to implement the call to the service

    ascend_msgs::PathOptimizerService path_optimizer_service;
    path_optimizer_service.request.pose = getCurrentPose();
    path_optimizer_service.request.twist = getCurrentTwist();
    path_optimizer_service.request.imu_data = sensor_msgs::Imu();
    path_optimizer_service.request.path = path;

    if (path_optimizer_client.call(path_optimizer_service)) {
        return path_optimizer_service.response.splines;
    }
    else {
        ROS_FATAL("Could not call path optimizer! Returning a spline containing the current position...");
        return Util::getSplineForSetpoint(getCurrentPose().pose.position, getCurrentPose().pose.position);
    }
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();
    auto spline = getSplineForPath(path);

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {

        ros::Duration current_time = ros::Time::now() - startTime;
        setpoint = Core::getControllerPtr()->getSetpoint(getPreferredController(), current_time.toSec(), spline);

        publishSetpoint();

        /*if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        } */        


        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}