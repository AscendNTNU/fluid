#include "trajectory.h"

fluid::Trajectory::Trajectory(const std::vector<geometry_msgs::Point>& path, 
                              const geometry_msgs::PoseStamped& pose,
                              const geometry_msgs::TwistStamped& twist,
                              const sensor_msgs::Imu& imu,
                              const double& target_speed, 
                              const double& curvature_gain) {
    /*
    std::vector<ascend_msgs::Spline> splines;

    ros::ServiceClient path_optimizer_client = node_handle.serviceClient<ascend_msgs::PathOptimizerService>("path_optimizer");
    ascend_msgs::PathOptimizerService path_optimizer_service;
    path_optimizer_service.request.pose = pose; 
    path_optimizer_service.request.twist = twist; 
    path_optimizer_service.request.imu_data = imu;
    path_optimizer_service.request.path = path;

    if (path_optimizer_client.call(path_optimizer_service)) {
        splines = path_optimizer_service.response.splines;
    }
    else {
        ROS_FATAL("Could not call path optimizer! Calculating a spline containing the current position...");
        splines = Util::getSplineForSetpoint(pose.pose.position, pose.pose.position);
    }

    // TODO: Do magic with the returned splines
    */

    // TODO: Temp, incorporate the spline service
    ros::ServiceClient generate_spline = node_handle.serviceClient<ascend_msgs::SplineService>("/control/spline_generator");

    ascend_msgs::SplineService spline_service_message;
    spline_service_message.request.waypoints_x = {1.0,  100, 200, 300, 400.0,  400.0, 400, 400, 400, 300, 200, 100, 0};
    spline_service_message.request.waypoints_y = {0.0,  0.0, 0.0, 0.0,   0.0,  -10.0, -20, -30, -40, -40, -40, -40, -40};
 
    spline_service_message.request.ds = 0.1;
    generate_spline.call(spline_service_message);

    const std::vector<double> speed_profile = calculateSpeedProfile(spline_service_message.response.cyaw, spline_service_message.response.ck, curvature_gain, target_speed);

    for (unsigned int i = 0; i < spline_service_message.response.cx.size(); i++) {
        double x         = spline_service_message.response.cx[i];
        double y         = spline_service_message.response.cy[i];
        double yaw       = Util::clampAngle(spline_service_message.response.cyaw[i]);
        double curvature = spline_service_message.response.ck[i];
        trajectory_points.push_back(TrajectoryPoint{x, y, speed_profile[i], yaw, curvature});
    }
}

std::vector<double> fluid::Trajectory::calculateSpeedProfile(const std::vector<double>& yaws, const std::vector<double>& curvatures, const double& curvature_gain, const double& target_speed) {
    std::vector<double> speed_profile(yaws.size(), target_speed);

    for (unsigned int i = 0; i < speed_profile.size() - 1; i++) {
        double curvature = 1 + curvature_gain * std::abs(curvatures[i]);
        speed_profile[i] = target_speed / curvature;
    }

    /*
    for (unsigned int i = speed_profile.size() - 1; i > speed_profile.size() - 10; i--) {
        speed_profile[i] = target_speed / (50 - i);

        if (speed_profile[i] <= 1.0 / 3.6) {
            speed_profile[i] = 1.0 / 3.6;
        }
    }
*/

    return speed_profile;
}

fluid::TrajectoryPointResult fluid::Trajectory::calculateNearestTrajectoryPoint(const geometry_msgs::Point& position) const {
    double shortest_distance = 1000000;
    unsigned int shortest_distance_index = 0;

    for (unsigned int i = 0; i < trajectory_points.size(); i++) {

        double dx = position.x - trajectory_points[i].x;
        double dy = position.y - trajectory_points[i].y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < shortest_distance) {
           shortest_distance = distance; 
           shortest_distance_index = i;
        }
    }

    // Check if we are on the left or right side of the path
    double dx = trajectory_points[shortest_distance_index].x - position.x;
    double dy = trajectory_points[shortest_distance_index].y - position.y;
    double angle = Util::clampAngle(trajectory_points[shortest_distance_index].yaw - std::atan2(dy, dx));

    if (angle < 0) {
        shortest_distance *= -1;
    }

    return TrajectoryPointResult{trajectory_points[shortest_distance_index], shortest_distance};
}

std::vector<fluid::TrajectoryPoint> fluid::Trajectory::getTrajectoryPoints() const {
    return trajectory_points;
}