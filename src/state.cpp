//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ascend_msgs/SplineService.h>

#include <tf/tf.h>

#include <utility>

#include "util.h"
#include "core.h"
#include "PID.h"

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
    twist_subscriber = node_handle.subscribe("mavros/local_position/velocity_local", 
                                             Core::message_queue_size, 
                                             &State::twistCallback, 
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

std::vector<ascend_msgs::Spline> fluid::State::getSplinesForPath(const std::vector<geometry_msgs::Point>& path) {
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
    ros::Time last_frame_time = ros::Time::now();

    // TODO: temp, should be included from path
    std::vector<ascend_msgs::Spline> splines = getSplinesForPath(path);
    ascend_msgs::Spline current_spline = splines[0];

    fluid::PID yaw_regulator(1.0, 0.0, 0.0);
    const double speed = 1.0;
    fluid::Path path(speed);

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {

        node_handle.param<double>("yaw_kp", yaw_regulator.kp, 1.0);
        node_handle.param<double>("yaw_ki", yaw_regulator.ki, 0.0);
        node_handle.param<double>("yaw_kd", yaw_regulator.kd, 0.0);

        ROS_INFO_STREAM(yaw_regulator.kp << ", " << yaw_regulator.ki << ", " << yaw_regulator.kd);

        ros::Time current_time = ros::Time::now();
        double delta_time = (current_time - last_frame_time).toSec();
        last_frame_time = current_time;
/*
        for (auto spline : splines) {

            if (current_time.toSec() < spline.timestamp) {
                current_spline = spline;
            }
        }
*/

        if (getPreferredController() != ControllerType::Positional) {

            double dx = getCurrentTwist().twist.linear.x;
            double dy = getCurrentTwist().twist.linear.y;

            geometry_msgs::Point future_point = getCurrentPose().pose.position;
            future_point.x += dx;
            future_point.y += dy;

            PathPointResult future_path_point = path.calculateNearestPathPoint(future_point);

            // double error = -std::atan(future_path_point.error);
            double error = atan2(future_path_point.path_point.y - getCurrentPose().pose.position.y, 
                                 future_path_point.path_point.x - getCurrentPose().pose.position.x);

            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = yaw_regulator.getActuation(error, delta_time);
            setpoint.velocity.x = speed * std::cos(setpoint.yaw); 
            setpoint.velocity.y = speed * std::sin(setpoint.yaw);
            setpoint.velocity.z = 0.0;
            setpoint.coordinate_frame = 1; 

            mavros_msgs::PositionTarget yaw_target;
            yaw_target.yaw = error;

            visualizer.publish(getCurrentPose().pose, getCurrentTwist().twist, path, future_path_point.path_point, setpoint);
        }
        else {
            setpoint = Core::getControllerPtr()->getSetpoint(getPreferredController(), current_time.toSec(), current_spline);
        }

        publishSetpoint();

        /*

        geometry_msgs::Point spline_position;
        
        spline_position.x = Util::evaluatePolynomial(current_time.toSec(), current_spline.x);
        spline_position.y = Util::evaluatePolynomial(current_time.toSec(), current_spline.y);
        spline_position.z = Util::evaluatePolynomial(current_time.toSec(), current_spline.z);
*/
/*
        if (Util::distanceBetween(spline_position, getCurrentPose().pose.position) > 3.0) {
            // Replan
            startTime = ros::Time::now();
            splines = getSplinesForPath(path);
            current_spline = splines[0];
        }
*/
        /*if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        } */        

        fluid::Core::getStatusPublisherPtr()->status.path = this->path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}