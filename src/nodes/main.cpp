#include <ros/ros.h>

#include "core.h"
#include "server.h"

#include <dynamic_reconfigure/server.h>
#include <fluid/ServerConfig.h>

void callback(fluid::ServerConfig& config, uint32_t level) {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_server");

    ROS_INFO("Starting Fluid FSM.");

    ros::NodeHandle node_handle;

    std::string publishing_topic = std::string(argv[1]);

    node_handle.getParam("refresh_rate", Core::refresh_rate);
    node_handle.getParam("auto_arm", Core::auto_arm);
    node_handle.getParam("auto_offboard", Core::auto_set_offboard);

    node_handle.getParam("distance_completion_threshold", Core::distance_completion_threshold);
    node_handle.getParam("velocity_completion_threshold", Core::velocity_completion_threshold);
    node_handle.getParam("yaw_completion_threshold", Core::yaw_completion_threshold);
    node_handle.getParam("default_height", Core::default_height);

    dynamic_reconfigure::Server<fluid::ServerConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<fluid::ServerConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    dynamic_reconfigure_server.setCallback(f);

    ros::spinOnce();

    Server server;
    server.start();

    return 0;
}
