#include "LQR.h"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "core.h"


fluid::LQR::LQR() {
    Q(0, 0) = 10.0;
}

Eigen::Matrix<double, 5, 5> fluid::LQR::solveDARE(const Eigen::Matrix<double, 5, 5>& A, 
                                                  const Eigen::Matrix<double, 5, 2>& B,
                                                  const Eigen::Matrix<double, 5, 5>& Q,
                                                  const Eigen::Matrix<double, 2, 2>& R) const {
    Eigen::Matrix<double, 5, 5> x      = Q;
    Eigen::Matrix<double, 5, 5> x_next = Q;
    const unsigned int max_iteration = 150;
    const double epsillon = 0.01;

    for (unsigned int i = 0; i < max_iteration; i++) {
        x_next = A.transpose() * x * A - 
                 A.transpose() * x * B * (R + B.transpose() * x * B).inverse() * B.transpose() * x * A + 
                 Q;

        if ((x_next.array().abs() - x.array().abs()).maxCoeff() < epsillon) {
            break;
        }

        x = x_next;
    }

    return x_next;
}

Eigen::Matrix<double, 2, 5> fluid::LQR::dlqr(const Eigen::Matrix<double, 5, 5>& A, 
                                             const Eigen::Matrix<double, 5, 2>& B,
                                             const Eigen::Matrix<double, 5, 5>& Q,
                                             const Eigen::Matrix<double, 2, 2>& R) const {

    Eigen::Matrix<double, 5, 5> X = solveDARE(A, B, Q, R);
    Eigen::Matrix<double, 2, 5> K = (R + B.transpose() * X * B).inverse() * (B.transpose() * X * A);

    return K;
}

fluid::Result fluid::LQR::control_law(geometry_msgs::Pose pose, 
                                      geometry_msgs::Twist twist, 
                                      Path path, 
                                      double previous_error, 
                                      double previous_error_in_yaw, 
                                      std::vector<double> speed_profile) {


    // TODO: Fix
    const PathPointResult path_point_result = path.calculateNearestPathPoint(geometry_msgs::Point());

    const double target_speed     = path_point_result.path_point.speed; 
    const double curvature        = path_point_result.path_point.curvature;
    const double current_speed    = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
    const double delta_time       = 1.0 / fluid::Core::refresh_rate;

    tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    
    if (!std::isfinite(yaw)) {
        yaw = 0;
    }

    /*
    double error_in_yaw = clampAngle(yaw - path.yaw[nearest_index_result.index]);

    Eigen::Matrix<double, 5, 5> A = Eigen::Matrix<double, 5, 5>::Zero(5, 5);
    A(0, 0) = 1.0;
    A(0, 1) = delta_time;
    A(1, 2) = current_speed;
    A(2, 2) = 1.0;
    A(2, 3) = delta_time;
    A(4, 4) = 1.0;

    Eigen::Matrix<double, 5, 2> B = Eigen::Matrix<double, 5, 2>::Zero(5, 2);
    B(3, 0) = current_speed;
    B(4, 1) = delta_time;

    Eigen::Matrix<double, 2, 5> K = dlqr(A, B, Q, R);


    // State vector
    Eigen::Matrix<double, 5, 1> x;
    x(0, 0) = nearest_index_result.error;
    x(1, 0) = (nearest_index_result.error - previous_error) / delta_time;
    x(2, 0) = error_in_yaw;                                                 // Linearization, assumue that sin(error_in_yaw) = error_in_yaw for small errors
    x(3, 0) = (error_in_yaw - previous_error_in_yaw) / delta_time;
    x(4, 0) = current_speed - target_speed;


    Eigen::Matrix<double, 2, 1> u = -(K * x);

    double feed_forward_steering_angle = std::atan2(curvature, 1);
    double feedback_steering_angle     = clampAngle(u(0, 0));

    double angle                       = feedback_steering_angle;
    double acceleration                = u(1, 0);

    return Result {angle, path.x[nearest_index_result.index], path.y[nearest_index_result.index], nearest_index_result.error, error_in_yaw, path.yaw[nearest_index_result.index], acceleration};
    */

   return Result{0, 0, 0, 0, 0, 0, 0};
}