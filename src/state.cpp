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
#include "LQR.h"

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

std::vector<double> calculateSpeedProfile(const std::vector<double>& yaws, const double& target_speed) {
    std::vector<double> speed_profile(yaws.size(), target_speed);

    double direction = 1.0;

    for (unsigned int i = 0; i < speed_profile.size() - 1; i++) {
        double delta_yaw = std::abs(yaws[i + 1] - yaws[i]);
        bool toggle = M_PI / 4.0 <= delta_yaw < M_PI / 2.0;

        if (toggle) {
            direction *= -1;
        }

        if (direction != 1) {
            speed_profile[i] = -target_speed;
        }
        else {
            speed_profile[i] = target_speed;
        }
    }

    for (unsigned int i = speed_profile.size() - 1; i > speed_profile.size() - 40; i--) {
        speed_profile[i] = target_speed / (50 - i);

        if (speed_profile[i] <= 1.0 / 3.6) {
            speed_profile[i] = 1.0 / 3.6;
        }
    }

    return speed_profile;
}


void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();
    std::vector<ascend_msgs::Spline> splines = getSplinesForPath(path);
    ascend_msgs::Spline current_spline = splines[0];


    ros::ServiceClient generate_spline = node_handle.serviceClient<ascend_msgs::SplineService>("/control/spline_generator");

    ascend_msgs::SplineService spline_service_message;
    spline_service_message.request.waypoints_x = {0.0,  6.0,  12.5, 10.0, 17.5, 20.0, 25.0};
    spline_service_message.request.waypoints_y = {0.0, -3.0, -5.0,   6.5,  3.0,  0.0,  0.0};
    spline_service_message.request.ds = 0.1;
    generate_spline.call(spline_service_message);

    auto spline_response = spline_service_message.response; 
    double e, e_th;

    e = e_th = 0;

    fluid::LQR lqr;
    ros::Publisher spline_path_publisher = node_handle.advertise<visualization_msgs::Marker>("spline_path", 10);
    ros::Publisher target_odometry_publisher = node_handle.advertise<visualization_msgs::Marker>("target_odometry", 10);


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

        for (unsigned int i = 0; i < spline_response.cx.size(); ++i) {

            geometry_msgs::Point p;
            p.x = spline_response.cx[i];
            p.y = spline_response.cy[i]; 
            p.z = 1.0;

            spline_path.points.push_back(p);
        }


        spline_path_publisher.publish(spline_path);

        ros::Duration current_time = ros::Time::now() - startTime;
/*
        for (auto spline : splines) {

            if (current_time.toSec() < spline.timestamp) {
                current_spline = spline;
            }
        }
*/
        if (getPreferredController() != ControllerType::Positional) {

            fluid::Path path{spline_response.cx, spline_response.cy, spline_response.cyaw, spline_response.ck};
            fluid::Result result = lqr.control_law(getCurrentPose().pose, getCurrentTwist().twist, path, e, e_th, calculateSpeedProfile(path.yaw, 10.0 / 3.6));

            e = result.error;
            e_th = result.error_in_yaw;

            tf2::Quaternion quaternion(getCurrentPose().pose.orientation.x, getCurrentPose().pose.orientation.y, getCurrentPose().pose.orientation.z, getCurrentPose().pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
            
            if (!std::isfinite(yaw)) {
                yaw = 0;
            }
 
            setpoint.type_mask = TypeMask::Velocity;
            setpoint.yaw = result.target_yaw; 
            setpoint.velocity.x += result.acceleration * std::cos(M_PI - setpoint.yaw) * 0.05;
            setpoint.velocity.y += result.acceleration * std::sin(M_PI - setpoint.yaw) * 0.05;
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
            target_odometry.scale.x = 1.0;
            target_odometry.scale.y = 0.05;
            target_odometry.scale.z = 0.05;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, setpoint.yaw);

            target_odometry.pose.orientation = tf2::toMsg(quat);

            target_odometry.color.a = 1.0;
            target_odometry.color.b = 1.0;
            target_odometry_publisher.publish(target_odometry);

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