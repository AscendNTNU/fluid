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

    fluid::PID yaw_regulator(1.0, 0.0, 0.0);
    fluid::Controller controller(yaw_regulator);

    const double target_speed = 20.0, curvature_gain = 10.0;
    fluid::Trajectory trajectory(path, getCurrentPose(), getCurrentTwist(), current_imu, target_speed, curvature_gain);

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {
        
        // The state will not set the setpoint itself, we issue custom controller
        if (!is_relative) {
            controller.pid.kp = fluid::Core::yaw_kp;
            controller.pid.ki = fluid::Core::yaw_ki;
            controller.pid.kd = fluid::Core::yaw_kd;

            ros::Time current_time = ros::Time::now();
            double delta_time = (current_time - last_frame_time).toSec();
            last_frame_time = current_time;

            double error = 0;
            TrajectoryPoint following_trajectory_point;
            setpoint = controller.getSetpoint(trajectory, getCurrentPose().pose, getCurrentTwist().twist, delta_time, error, following_trajectory_point);

            mavros_msgs::PositionTarget yaw_target;
            yaw_target.yaw = error;

            visualizer.publish(getCurrentPose().pose, getCurrentTwist().twist, trajectory, following_trajectory_point, setpoint);
        }

        publishSetpoint();

        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}