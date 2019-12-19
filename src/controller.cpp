#include "controller.h"
#include "util.h"
#include "type_mask.h"
#include "core.h"

fluid::Controller::Controller(const PID& pid) : pid(pid) {}

mavros_msgs::PositionTarget fluid::Controller::getSetpoint(const Trajectory& trajectory,
                                                           const geometry_msgs::Pose& pose,
                                                           const geometry_msgs::Twist& twist,
                                                           const double& delta_time,
                                                           double& out_error,
                                                           TrajectoryPoint& out_following_trajectory_point) {

    double vdx = twist.linear.x;
    double vdy = twist.linear.y;

    // Find the current position closest to the path, apply the velocity vector in that direction from the
    // path's yaw and set that as the following point
    double velocity = sqrt(vdx*vdx + vdy*vdy);
    TrajectoryPointResult current_trajectory_point = trajectory.calculateNearestTrajectoryPoint(pose.position);
    out_following_trajectory_point = trajectory.getTrajectoryPoints()[std::min(trajectory.calculateNearestTrajectoryPoint(pose.position).index + 300, 
                                                                               static_cast<unsigned int>(trajectory.getTrajectoryPoints().size()) - 1)];

    tf2::Quaternion quat(pose.orientation.x, 
                         pose.orientation.y, 
                         pose.orientation.z, 
                         pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
    // it to zero. 
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    // out_error = -std::atan(current_trajectory_point.error);
    out_error = atan2(out_following_trajectory_point.y - pose.position.y, 
                      out_following_trajectory_point.x - pose.position.x);

    double steering_yaw = pid.getActuation(out_error, delta_time);

    mavros_msgs::PositionTarget setpoint;
    setpoint.type_mask = TypeMask::Velocity;
    setpoint.yaw = steering_yaw; 
    setpoint.velocity.x = out_following_trajectory_point.speed * std::cos(setpoint.yaw); 
    setpoint.velocity.y = out_following_trajectory_point.speed * std::sin(setpoint.yaw);
    setpoint.velocity.z = 0.0;
    setpoint.coordinate_frame = 1; 

    return setpoint;
}
