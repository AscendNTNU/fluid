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
    bool ekf, use_perception, should_auto_arm, should_auto_offboard, interaction_interacts, interaction_show_prints;
    float distance_completion_threshold, velocity_completion_threshold, default_height;
    float interact_max_vel, interact_max_acc, travel_speed, travel_accel, travel_max_angle;
    float* fh_offset = (float*) calloc(3,sizeof(float));
    
    if (!node_handle.getParam(prefix + "ekf", ekf)) {
        exitAtParameterExtractionFailure(prefix + "ekf");
    }

    if (!node_handle.getParam(prefix + "use_perception", use_perception)) {
        exitAtParameterExtractionFailure(prefix + "use_perception");
    }

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

    if (!node_handle.getParam(prefix + "interaction_interacts", interaction_interacts)) {
        exitAtParameterExtractionFailure(prefix + "interaction_interacts");
    }

    if (!node_handle.getParam(prefix + "interaction_show_prints", interaction_show_prints)) {
        exitAtParameterExtractionFailure(prefix + "interaction_show_prints");
    }
    
    if (!node_handle.getParam(prefix + "interaction_max_vel", interact_max_vel)) {
        exitAtParameterExtractionFailure(prefix + "interaction_max_vel");
    }

    if (!node_handle.getParam(prefix + "interaction_max_acc", interact_max_acc)) {
        exitAtParameterExtractionFailure(prefix + "interaction_max_acc");
    }

    if (!node_handle.getParam(prefix + "travel_max_angle", travel_max_angle)) {
        exitAtParameterExtractionFailure(prefix + "travel_max_angle");
    }

    if (!node_handle.getParam(prefix + "fh_offset_x", fh_offset[0])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_x");
    }

    if (!node_handle.getParam(prefix + "fh_offset_y", fh_offset[1])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_y");
    }

    if (!node_handle.getParam(prefix + "fh_offset_z", fh_offset[2])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_z");
    }

    if (!node_handle.getParam(prefix + "travel_speed", travel_speed)) {
        exitAtParameterExtractionFailure(prefix + "travel_speed");
    }

    if (!node_handle.getParam(prefix + "travel_accel", travel_accel)) {
        exitAtParameterExtractionFailure(prefix + "travel_accel");
    }
    FluidConfiguration configuration{ekf,
                                    use_perception,
                                    refresh_rate,
                                    should_auto_arm,
                                    should_auto_offboard,
                                    distance_completion_threshold,
                                    velocity_completion_threshold,
                                    default_height,
                                    interaction_interacts,
                                    interaction_show_prints,
                                    interact_max_vel,
                                    interact_max_acc,
                                    travel_max_angle,
                                    fh_offset,
                                    travel_speed,
                                    travel_accel
                                    };

    Fluid::initialize(configuration);

    Fluid::getInstance().run();

    return 0;
}
