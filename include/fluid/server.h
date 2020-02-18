#ifndef SERVER_H
#define SERVER_H

#include <actionlib/server/simple_action_server.h>
#include <ascend_msgs/FluidAction.h>
#include <ascend_msgs/PositionYawTarget.h>
#include <ros/ros.h>

#include "operation.h"

/** 
 *  \brief Encapsulates a ROS action server which performs operations based on requests.
 */
class Server {
    typedef actionlib::SimpleActionServer<ascend_msgs::FluidAction> ActionlibServer;

private:
    ros::NodeHandle node_handle_;
    ActionlibServer actionlib_server_;

    std::shared_ptr<Operation> retrieveNewOperation();
    void preemptCallback();

public:
    Server();
    void start();
};

#endif
