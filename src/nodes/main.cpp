#include <ros/ros.h>

#include "fluid.h"

void exitAtParameterExtractionFailure(const std::string& param) {
    ROS_FATAL_STREAM(ros::this_node::getName() << ": Could not find parameter: " << param.c_str());
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_server");

    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Starting up.");

    ros::NodeHandle node_handle;
    const std::string prefix = ros::this_node::getName() + "/";
    int refresh_rate;
    bool should_auto_arm, should_auto_offboard;
    float distance_completion_threshold, velocity_completion_threshold, default_height;
    float* LQR_gains = (float*) calloc(4,sizeof(float));

    if (!node_handle.getParam(prefix + "refresh_rate", refresh_rate)) {
        exitAtParameterExtractionFailure(prefix + "refresh_rate");
    }

    if (!node_handle.getParam(prefix + "should_auto_arm", should_auto_arm)) {
        exitAtParameterExtractionFailure(prefix + "should_auto_arm");
    }

    if (!node_handle.getParam(prefix + "should_auto_offboard", should_auto_offboard)) {
        exitAtParameterExtractionFailure(prefix + "should_auto_offboard");
    }

    if (!node_handle.getParam(prefix + "distance_completion_threshold", distance_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "distance_completion_threshold");
    }

    if (!node_handle.getParam(prefix + "velocity_completion_threshold", velocity_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "velocity_completion_threshold");
    }

    if (!node_handle.getParam(prefix + "default_height", default_height)) {
        exitAtParameterExtractionFailure(prefix + "default_height");
    }

    if (!node_handle.getParam(prefix + "Kpx", LQR_gains[0])) {
        exitAtParameterExtractionFailure(prefix + "Kpx");
    }

    if (!node_handle.getParam(prefix + "Kpy", LQR_gains[1])) {
        exitAtParameterExtractionFailure(prefix + "Kpy");
    }

    if (!node_handle.getParam(prefix + "Kvx", LQR_gains[2])) {
        exitAtParameterExtractionFailure(prefix + "Kvx");
    }

    if (!node_handle.getParam(prefix + "Kvy", LQR_gains[3])) {
        exitAtParameterExtractionFailure(prefix + "Kvy");
    }
    FluidConfiguration configuration{refresh_rate,
                                     should_auto_arm,
                                     should_auto_offboard,
                                     distance_completion_threshold,
                                     velocity_completion_threshold,
                                     default_height,
                                     LQR_gains};

    Fluid::initialize(configuration);

    Fluid::getInstance().run();

    return 0;
}
