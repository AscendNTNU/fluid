//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ascend_msgs/SplineService.h>
#include <ascend_msgs/LQR.h>

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
    // std::vector<ascend_msgs::Spline> splines = getSplinesForPath(path);
    // ascend_msgs::Spline current_spline = splines[0];

    fluid::PID yaw_regulator(1.0, 0.5, 0.0);
    ros::Publisher spline_path_publisher = node_handle.advertise<visualization_msgs::Marker>("spline_path", 10);
    ros::Publisher target_odometry_publisher = node_handle.advertise<visualization_msgs::Marker>("target_odometry", 10);
    ros::Publisher nearest_point_publisher = node_handle.advertise<visualization_msgs::Marker>("nearest_point", 10);

    fluid::Path path;

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {

        visualization_msgs::Marker spline_path;
        spline_path.header.frame_id = "/map";
        spline_path.header.stamp = ros::Time::now();
        spline_path.action = visualization_msgs::Marker::ADD;
        spline_path.id = 0;
        spline_path.type = visualization_msgs::Marker::LINE_STRIP;
        spline_path.scale.x = 0.05;
        spline_path.color.b = 1.0;
        spline_path.color.a = 1.0;

        for (unsigned int i = 0; i < path.x_values.size(); ++i) {

            geometry_msgs::Point p;
            p.x = spline_response.cx[i];
            p.y = spline_response.cy[i]; 
            p.z = 1.0;

            spline_path.points.push_back(p);
        }


        spline_path_publisher.publish(spline_path);

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


            tf2::Quaternion quaternion(getCurrentPose().pose.orientation.x, getCurrentPose().pose.orientation.y, getCurrentPose().pose.orientation.z, getCurrentPose().pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
            
            if (!std::isfinite(yaw)) {
                yaw = 0;
            }
 
            double dx = getCurrentTwist().twist.linear.x;
            double dy = getCurrentTwist().twist.linear.y;
           
            geometry_msgs::Pose future_pose = getCurrentPose().pose;
            future_pose.position.x += dx;
            future_pose.position.y += dy; 
            NearestStateIndexResult result = calculateNearestIndex(future_pose, path);

            ROS_INFO_STREAM(dx << ", " << dy);


            double speed = 1.0;
            double error = std::atan(result.error);

 
            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = -pid.getActuation(error, delta_time);
            setpoint.velocity.x = speed * std::cos(setpoint.yaw); 
            setpoint.velocity.y = speed * std::sin(setpoint.yaw);
            setpoint.velocity.z = 0.0;
            setpoint.coordinate_frame = 1; 

            visualization_msgs::Marker target_odometry;
            target_odometry.header.frame_id = "odom";
            target_odometry.header.stamp = ros::Time();
            target_odometry.id = 1;
            target_odometry.type = visualization_msgs::Marker::ARROW;
            target_odometry.action = visualization_msgs::Marker::ADD;
            target_odometry.pose.position.x = getCurrentPose().pose.position.x;
            target_odometry.pose.position.y = getCurrentPose().pose.position.y;
            target_odometry.pose.position.z = getCurrentPose().pose.position.z;
            target_odometry.scale.x = speed;
            target_odometry.scale.y = 0.05;
            target_odometry.scale.z = 0.05;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, setpoint.yaw);

            target_odometry.pose.orientation = tf2::toMsg(quat);

            target_odometry.color.a = 1.0;
            target_odometry.color.b = 1.0;

            target_odometry_publisher.publish(target_odometry);

            visualization_msgs::Marker nearest_point;
            nearest_point.header.frame_id = "map";
            nearest_point.header.stamp = ros::Time();
            nearest_point.id = 2;
            nearest_point.type = visualization_msgs::Marker::SPHERE;
            nearest_point.action = visualization_msgs::Marker::ADD;
            nearest_point.pose.position.x = path.x[result.index]; 
            nearest_point.pose.position.y = path.y[result.index];
            nearest_point.pose.position.z = 1.0;
            nearest_point.scale.x = 0.5;
            nearest_point.scale.y = 0.5;
            nearest_point.scale.z = 0.5;
            nearest_point.color.a = 1.0;
            nearest_point.color.r = 1.0;
            nearest_point.color.g = 1.0;

            nearest_point_publisher.publish(nearest_point);

       }
        else {
            setpoint = Core::getControllerPtr()->getSetpoint(getPreferredController(), current_time.toSec(), current_spline);
        }

        publishSetpoint();

        geometry_msgs::Point spline_position;
        
        spline_position.x = Util::evaluatePolynomial(current_time.toSec(), current_spline.x);
        spline_position.y = Util::evaluatePolynomial(current_time.toSec(), current_spline.y);
        spline_position.z = Util::evaluatePolynomial(current_time.toSec(), current_spline.z);

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

        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}