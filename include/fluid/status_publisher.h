/**
 * @file status_publisher.h
 */

#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <ascend_msgs/FluidStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/**
 * @brief Publishes information about Fluid and visualization of paths the drone is going to fly/have flown to rviz.
 */
class StatusPublisher {
   private:
    /**
     * @brief Sets up the publishers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief Used to create a path of where the drone has been.
     */
    ros::Subscriber pose_subscriber;

    /**
     * @brief Publishes status, trace of where the drone has been and current setpoint as as a visualization marker.
     */
    ros::Publisher status_publisher, trace_publisher, setpoint_marker_publisher;

    /**
     * @brief Retrieves the pose of the drone.
     *
     * @param pose_ptr Current pose.
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr);

    /**
     * @brief The trace path.
     */
    nav_msgs::Path trace_path;

    /**
     * @brief Marker visualizing the setpoint.
     */
    visualization_msgs::Marker setpoint_marker;

   public:
    /**
     * @brief Status message.
     */
    ascend_msgs::FluidStatus status;

    /**
     * @brief Sets up the subscribers and publishers.
     */
    StatusPublisher();

    /**
     * @brief Publishes the current status, trace and setpoint marker.
     */
    void publish();
};

#endif