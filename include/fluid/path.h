#ifndef FLUID_PATH_H
#define FLUID_PATH_H

#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <ascend_msgs/SplineService.h>

#include "util.h"

namespace fluid {

    struct PathPoint {
        double x, y, speed, yaw, curvature;
    };

    struct PathPointResult {
        const PathPoint path_point;
        const double error;
    };

    class Path {

        private:

            ros::NodeHandle node_handle;
            
            std::vector<double> calculateSpeedProfile(const std::vector<double>& yaws, const std::vector<double>& curvatures, const double& curvature_gain, const double& target_speed);

            std::vector<fluid::PathPoint> path_points;

        public:

            Path(const double& target_speed, const double& curvature_gain);

            PathPointResult calculateNearestPathPoint(const geometry_msgs::Point& position) const;

            std::vector<fluid::PathPoint> getPathPoints() const;

    };
}

#endif