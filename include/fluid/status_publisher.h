#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <ascend_msgs/FluidStatus.h>
#include <ros/ros.h>

class StatusPublisher {
private:
    ros::NodeHandle node_handle;

    ros::Publisher publisher;

public:
    ascend_msgs::FluidStatus status;

    StatusPublisher();

    void publish();
};

#endif