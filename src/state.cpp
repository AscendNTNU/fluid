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
                    bool should_check_obstalce_avoidance_completion,
                    bool is_relative) : 

					identifier(identifier),
                    px4_mode(px4_mode),
                    steady(steady), 
                    should_check_obstacle_avoidance_completion(should_check_obstacle_avoidance_completion),
                    is_relative(is_relative) {

    pose_subscriber = node_handle.subscribe("mavros/local_position/pose", 1, &State::poseCallback, this);
    twist_subscriber = node_handle.subscribe("mavros/local_position/velocity_local", 1, &State::twistCallback, this);
    imu_subscriber = node_handle.subscribe("mavros/imu/data", 1, &State::imuCallback, this);

    setpoint_publisher = node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", Core::message_queue_size);
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


void fluid::State::imuCallback(const sensor_msgs::ImuConstPtr imu) {
    current_imu = *imu;
}

void fluid::State::publishSetpoint() {
    setpoint_publisher.publish(setpoint);
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();
    ros::Time last_frame_time = ros::Time::now();

    fluid::Trajectory trajectory(path, 
                                 getCurrentPose(), 
                                 getCurrentTwist(), 
                                 current_imu, 
                                 Core::getControllerConfig().target_speed, 
                                 Core::getControllerConfig().curvature_gain);

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {
        
        // The state will not set the setpoint itself, we issue custom controller
        if (!is_relative) {

            fluid::Trajectory sub_trajectory = trajectory.generateSubTrajectory(getCurrentPose().pose.position, 200);

            ros::Time current_time = ros::Time::now();
            double delta_time = (current_time - last_frame_time).toSec();
            last_frame_time = current_time;

            double out_error = 0;
            TrajectoryPoint out_following_trajectory_point;
            setpoint = Core::getControllerPtr()->getSetpoint(sub_trajectory, 
                                                             getCurrentPose().pose, 
                                                             getCurrentTwist().twist, 
                                                             delta_time, 
                                                             out_error, 
                                                             out_following_trajectory_point);

            mavros_msgs::PositionTarget yaw_target;
            yaw_target.yaw = out_error;

            visualizer.publish(getCurrentPose().pose, 
                               getCurrentTwist().twist,
                               trajectory, 
                               sub_trajectory, 
                               out_following_trajectory_point, 
                               setpoint);
        }

        publishSetpoint();

        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}