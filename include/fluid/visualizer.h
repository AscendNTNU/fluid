#ifndef FLUID_VISUALIZER_H
#define FLUID_VISUALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <mavros_msgs/PositionTarget.h>

#include "path.h"

namespace fluid {

    /**
     * Publishes markers based on current odometry and path for RVIZ
     */
    class Visualizer {

        private:

            ros::NodeHandle node_handle;

            ros::Publisher path_publisher; 
            ros::Publisher target_odometry_publisher;
            ros::Publisher target_point_on_path_publisher;

        public:

            Visualizer();

            void publish(const geometry_msgs::Pose& pose, 
                         const geometry_msgs::Twist& twist, 
                         const Path& path, 
                         const PathPoint& path_point, 
                         const mavros_msgs::PositionTarget& setpoint);
    };
}

#endif