#include "path.h"

fluid::Path::Path(const double& target_speed) {

    ros::NodeHandle node_handle;
    ros::ServiceClient generate_spline = node_handle.serviceClient<ascend_msgs::SplineService>("/control/spline_generator");

    ascend_msgs::SplineService spline_service_message;
    spline_service_message.request.waypoints_x = {1.0,  6.0,  12.5, 10.0, 17.5, 20.0, 25.0};
    spline_service_message.request.waypoints_y = {0.0, -3.0, -5.0,   6.5,  3.0,  0.0,  0.0};
    spline_service_message.request.ds = 0.1;
    generate_spline.call(spline_service_message);

    const std::vector<double> speed_profile = calculateSpeedProfile(spline_service_message.response.cyaw, target_speed);

    for (unsigned int i = 0; i < spline_service_message.response.cx.size(); i++) {
        double x         = spline_service_message.response.cx[i];
        double y         = spline_service_message.response.cx[i];
        double yaw       = clampAngle(spline_service_message.response.cyaw[i]);
        double curvature = spline_service_message.response.ck[i];
        path_points.push_back(PathPoint{x, y, speed_profile[i], yaw, curvature});
    }
}

double fluid::Path::clampAngle(double angle) const {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

std::vector<double> fluid::Path::calculateSpeedProfile(const std::vector<double>& yaws, const double& target_speed) {
    std::vector<double> speed_profile(path_points.size(), target_speed);

    double direction = 1.0;

    for (unsigned int i = 0; i < speed_profile.size() - 1; i++) {
        double delta_yaw = std::abs(path_points[i + 1].yaw - path_points[i].yaw);
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

fluid::PathPointResult fluid::Path::calculateNearestPathPoint(const geometry_msgs::Point& position) const {
    double shortest_distance = 1000000;
    unsigned int shortest_distance_index = 0;

    for (unsigned int i = 0; i < path_points.size(); i++) {

        double dx = position.x - path_points[i].x;
        double dy = position.y - path_points[i].y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < shortest_distance) {
           shortest_distance = distance; 
           shortest_distance_index = i;
        }
    }

    return PathPointResult{path_points[shortest_distance_index], shortest_distance};
}

std::vector<fluid::PathPoint> fluid::Path::getPathPoints() const {
    return path_points;
}