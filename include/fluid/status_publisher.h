#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <ascend_msgs/FluidStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

/**
 * \brief Publishes information about Fluid and visualization of paths the drone is going to fly/have flown to rviz.
 */
class StatusPublisher {
   private:
    ros::NodeHandle node_handle;

    ros::Subscriber pose_subscriber;
    ros::Publisher status_publisher, trace_publisher;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose_ptr);

    nav_msgs::Path trace_path;

   public:
    ascend_msgs::FluidStatus status;

    StatusPublisher();

    void publish();
};

#endif