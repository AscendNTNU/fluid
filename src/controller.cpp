#include "controller.h"
#include "util.h"
#include "type_mask.h"
#include "core.h"

fluid::Controller::Controller(const PID& pid) : pid(pid) {}

mavros_msgs::PositionTarget fluid::Controller::getSetpoint(const Trajectory& trajectory,
                                                           const geometry_msgs::Pose& pose,
                                                           const geometry_msgs::Twist& twist,
                                                           const double& delta_time) {

    double dx = twist.linear.x;
    double dy = twist.linear.y;

    // Find the current position closest to the path, apply the velocity vector in that direction from the
    // path's yaw and set that as the following point
    double velocity = sqrt(dx*dx + dy*dy);
    ROS_INFO_STREAM(velocity);
    TrajectoryPoint current_trajectory_point, following_trajectory_point;
    current_trajectory_point = following_trajectory_point = trajectory.calculateNearestTrajectoryPoint(pose.position).trajectory_point;
    following_trajectory_point.x += velocity * cos(current_trajectory_point.yaw);
    following_trajectory_point.y += velocity * sin(current_trajectory_point.yaw);

    geometry_msgs::Point point;
    point.x = following_trajectory_point.x;
    point.y = following_trajectory_point.y;
    following_trajectory_point = trajectory.calculateNearestTrajectoryPoint(point).trajectory_point;

    tf2::Quaternion quat(pose.orientation.x, 
                         pose.orientation.y, 
                         pose.orientation.z, 
                         pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
    // it to zero. 
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    // double error = -std::atan(future_path_point.error);
    double error = atan2(following_trajectory_point.y - pose.position.y, 
                         following_trajectory_point.x - pose.position.x);

    double steering_yaw = pid.getActuation(error, delta_time);

    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Velocity;
    setpoint.yaw = steering_yaw; 
    setpoint.velocity.x = following_trajectory_point.speed * std::cos(setpoint.yaw); 
    setpoint.velocity.y = following_trajectory_point.speed * std::sin(setpoint.yaw);
    setpoint.velocity.z = 0.0;
    setpoint.coordinate_frame = 1; 

    return setpoint;
}
