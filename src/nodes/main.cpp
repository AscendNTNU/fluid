#include <ros/ros.h>

#include "core.h"
#include "server.h"

#include <dynamic_reconfigure/server.h>
#include <fluid/ServerConfig.h>

void callback(fluid::ServerConfig& config, uint32_t level) {}

void exitAtParameterExtractionFailure(const std::string& param) {
    ROS_FATAL_STREAM("Could not find parameter: " << param);
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_server");

    ROS_INFO("Starting Fluid FSM.");

    ros::NodeHandle node_handle;
    const std::string prefix = ros::this_node::getName() + "/";

    if (!node_handle.getParam(prefix + "refresh_rate", Core::refresh_rate)) {
        exitAtParameterExtractionFailure(prefix + "refresh_rate");
    }

    if (!node_handle.getParam(prefix + "auto_arm", Core::auto_arm)) {
        exitAtParameterExtractionFailure(prefix + "auto_arm");
    }

    if (!node_handle.getParam(prefix + "auto_offboard", Core::auto_set_offboard)) {
        exitAtParameterExtractionFailure(prefix + "auto_offboard");
    }

    if (!node_handle.getParam(prefix + "distance_completion_threshold", Core::distance_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "distance_completion_threshold");
    }

    if (!node_handle.getParam(prefix + "velocity_completion_threshold", Core::velocity_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "velocity_completion_threshold");
    }

    if (!node_handle.getParam(prefix + "yaw_completion_threshold", Core::yaw_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "yaw_completion_threshold");
    }

    if (!node_handle.getParam(prefix + "default_height", Core::default_height)) {
        exitAtParameterExtractionFailure(prefix + "default_height");
    }

    dynamic_reconfigure::Server<fluid::ServerConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<fluid::ServerConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    dynamic_reconfigure_server.setCallback(f);

    ros::spinOnce();

    Server server;
    server.start();

    return 0;
}
