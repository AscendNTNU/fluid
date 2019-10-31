#include "visualizer.h"

fluid::Visualizer::Visualizer() {
        path_publisher                 = node_handle.advertise<visualization_msgs::Marker>("spline_path", 1);
        target_odometry_publisher      = node_handle.advertise<visualization_msgs::Marker>("target_odometry", 1);
        target_point_on_path_publisher = node_handle.advertise<visualization_msgs::Marker>("target_point", 1);
}


void fluid::Visualizer::publish(const geometry_msgs::Pose& pose, 
                                const geometry_msgs::Twist& twist, 
                                const Path& path, 
                                const PathPoint& path_point, 
                                const mavros_msgs::PositionTarget& setpoint) const {

    visualization_msgs::Marker target_odometry;
    target_odometry.header.frame_id = "odom";
    target_odometry.header.stamp = ros::Time();
    target_odometry.id = 1;
    target_odometry.type = visualization_msgs::Marker::ARROW;
    target_odometry.action = visualization_msgs::Marker::ADD;
    target_odometry.pose.position.x = pose.position.x;
    target_odometry.pose.position.y = pose.position.y;
    target_odometry.pose.position.z = pose.position.z;
    target_odometry.scale.x = path_point.speed;
    target_odometry.scale.y = 0.05;
    target_odometry.scale.z = 0.05;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, setpoint.yaw);
    target_odometry.pose.orientation = tf2::toMsg(quat);
    target_odometry.color.a = 1.0;
    target_odometry.color.b = 1.0;
    target_odometry_publisher.publish(target_odometry);


    visualization_msgs::Marker target_point_on_path;
    target_point_on_path.header.frame_id = "map";
    target_point_on_path.header.stamp = ros::Time();
    target_point_on_path.id = 2;
    target_point_on_path.type = visualization_msgs::Marker::SPHERE;
    target_point_on_path.action = visualization_msgs::Marker::ADD;
    target_point_on_path.pose.position.x = path_point.x; 
    target_point_on_path.pose.position.y = path_point.x;
    target_point_on_path.pose.position.z = 1.0;
    target_point_on_path.scale.x = 0.5;
    target_point_on_path.scale.y = 0.5;
    target_point_on_path.scale.z = 0.5;
    target_point_on_path.color.a = 1.0;
    target_point_on_path.color.r = 1.0;
    target_point_on_path.color.g = 1.0;

    target_point_on_path_publisher.publish(target_point_on_path);


    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "/map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.05;
    path_marker.color.b = 1.0;
    path_marker.color.a = 1.0;

    for (PathPoint path_point : path.getPathPoints()) {
        geometry_msgs::Point point;
        point.x = path_point.x;
        point.y = path_point.y;
        path_marker.points.push_back(point);
    }

    path_publisher.publish(path_marker);
}