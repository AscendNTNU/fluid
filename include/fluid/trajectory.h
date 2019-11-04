#ifndef FLUID_TRAJECTORY_H
#define FLUID_TRAJECTORY_H

#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <ascend_msgs/SplineService.h>
#include <ascend_msgs/PathOptimizerService.h>
#include <ascend_msgs/Spline.h>

#include "util.h"

namespace fluid {

    struct TrajectoryPoint {
        double x, y, speed, yaw, curvature;
    };

    struct TrajectoryPointResult {
        const TrajectoryPoint trajectory_point;
        const double error;
    };

    class Trajectory {

        private:

            ros::NodeHandle node_handle;
            
            std::vector<double> calculateSpeedProfile(const std::vector<double>& yaws, const std::vector<double>& curvatures, const double& curvature_gain, const double& target_speed);

            std::vector<fluid::TrajectoryPoint> trajectory_points;

        public:

            Trajectory(const std::vector<geometry_msgs::Point>& path, 
                       const geometry_msgs::PoseStamped& pose,
                       const geometry_msgs::TwistStamped& twist,
                       const sensor_msgs::Imu& imu,
                       const double& target_speed, 
                       const double& curvature_gain);

            TrajectoryPointResult calculateNearestTrajectoryPoint(const geometry_msgs::Point& position) const;

            std::vector<fluid::TrajectoryPoint> getTrajectoryPoints() const;

    };
}

#endif