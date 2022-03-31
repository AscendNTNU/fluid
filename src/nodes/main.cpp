#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "fluid/interact_operation.hpp"

void exitAtParameterExtractionFailure(const std::string& param) {
    RCLCPP_FATAL(rclcpp::get_logger("fluid/main"), ": Could not find parameter: ", param.c_str());
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fluid_server");
    RCLCPP_INFO(rclcpp::get_logger("fluid/main"), ": Starting up.");

    const std::string prefix = std::string(node->get_name()) + "/";
    int refresh_rate;
    bool ekf, use_perception, should_auto_arm, should_auto_offboard, interaction_show_prints;
    float distance_completion_threshold, velocity_completion_threshold, default_height, travel_max_angle;
    float interact_max_vel, interact_max_acc, travel_speed, travel_accel;
    float* fh_offset = (float*) calloc(3,sizeof(float));
    
    if (!node->get_parameter(prefix + "ekf", ekf)) {
        exitAtParameterExtractionFailure(prefix + "ekf");
    }

    if (!node->get_parameter(prefix + "use_perception", use_perception)) {
        exitAtParameterExtractionFailure(prefix + "use_perception");
    }

    if (!node->get_parameter(prefix + "refresh_rate", refresh_rate)) {
        exitAtParameterExtractionFailure(prefix + "refresh_rate");
    }

    if (!node->get_parameter(prefix + "should_auto_arm", should_auto_arm)) {
        exitAtParameterExtractionFailure(prefix + "should_auto_arm");
    }

    if (!node->get_parameter(prefix + "should_auto_offboard", should_auto_offboard)) {
        exitAtParameterExtractionFailure(prefix + "should_auto_offboard");
    }

    if (!node->get_parameter(prefix + "distance_completion_threshold", distance_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "distance_completion_threshold");
    }

    if (!node->get_parameter(prefix + "velocity_completion_threshold", velocity_completion_threshold)) {
        exitAtParameterExtractionFailure(prefix + "velocity_completion_threshold");
    }

    if (!node->get_parameter(prefix + "default_height", default_height)) {
        exitAtParameterExtractionFailure(prefix + "default_height");
    }

    if (!node->get_parameter(prefix + "interaction_show_prints", interaction_show_prints)) {
        exitAtParameterExtractionFailure(prefix + "interaction_show_prints");
    }
    
    if (!node->get_parameter(prefix + "interaction_max_vel", interact_max_vel)) {
        exitAtParameterExtractionFailure(prefix + "interaction_max_vel");
    }

    if (!node->get_parameter(prefix + "interaction_max_acc", interact_max_acc)) {
        exitAtParameterExtractionFailure(prefix + "interaction_max_acc");
    }

    if (!node->get_parameter(prefix + "travel_max_angle", travel_max_angle)) {
        exitAtParameterExtractionFailure(prefix + "travel_max_angle");
    }

    if (!node->get_parameter(prefix + "fh_offset_x", fh_offset[0])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_x");
    }

    if (!node->get_parameter(prefix + "fh_offset_y", fh_offset[1])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_y");
    }

    if (!node->get_parameter(prefix + "fh_offset_z", fh_offset[2])) {
        exitAtParameterExtractionFailure(prefix + "fh_offset_z");
    }

    if (!node->get_parameter(prefix + "travel_speed", travel_speed)) {
        exitAtParameterExtractionFailure(prefix + "travel_speed");
    }

    if (!node->get_parameter(prefix + "travel_accel", travel_accel)) {
        exitAtParameterExtractionFailure(prefix + "travel_accel");
    }
    MastNodeConfiguration configuration{ekf,
                                    use_perception,
                                    refresh_rate,
                                    should_auto_arm,
                                    should_auto_offboard,
                                    distance_completion_threshold,
                                    velocity_completion_threshold,
                                    default_height,
                                    interaction_show_prints,
                                    interact_max_vel,
                                    interact_max_acc,
                                    travel_max_angle,
                                    fh_offset,
                                    travel_speed,
                                    travel_accel
                                    };

    //InteractOperation interact = InteractOperation(0.0, false, true, configuration, 3.0);

    int rate_int = (int)configuration.refresh_rate;
    rclcpp::Rate rate(rate_int);
    auto interact_ptr = std::make_shared<InteractOperation>(InteractOperation(0.0, false, true, configuration, 3.0));
    interact_ptr->initialize();
    do {
        interact_ptr->tick();
        if (interact_ptr->autoPublish)
            interact_ptr->publishSetpoint();
        rclcpp::spin_some(interact_ptr);
        rate.sleep();
    } while (rclcpp::ok() || (!interact_ptr->hasFinishedExecution()));
    return 0;
}
