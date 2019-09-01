//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_SERVER_H
#define FLUID_FSM_SERVER_H

#include <ros/ros.h>
#include <ascend_msgs/FluidAction.h>
#include <actionlib/server/simple_action_server.h>

#include "operation.h"

namespace fluid {

    /** \class Server
     *  \brief Encapsulates a ROS action server which performs operations based on requests.
     */
    class Server {

        typedef actionlib::SimpleActionServer<ascend_msgs::FluidAction> ActionlibServer;

    private:

        ros::NodeHandle node_handle_;                                ///< Used to instantiate the ROS action server
        ActionlibServer actionlib_server_;                         

    public:

        Server();

        std::shared_ptr<fluid::Operation> retrieveNewOperation();
        void goalCallback();
        void preemptCallback();
        void start();
    };
}

#endif
