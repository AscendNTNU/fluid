#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "fluid/config.hpp"
#include "fluid/interact_operation.hpp"


extern bool ekf, use_perception, should_auto_arm, should_auto_offboard, interaction_show_prints;
extern float distance_completion_threshold, velocity_completion_threshold, default_height, travel_max_angle;
extern float interact_max_vel, interact_max_acc, travel_speed, travel_accel;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fluid_server");
    RCLCPP_INFO(rclcpp::get_logger("fluid/main"), "Starting up.");

    const std::string prefix = std::string(node->get_name()) + "/";
   
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
                                    fh_offset_x,
                                    fh_offset_y,
                                    fh_offset_z,
                                    travel_speed,
                                    travel_accel
                                    };

    int rate_int = (int)configuration.refresh_rate;
    rclcpp::Rate rate(rate_int);


    auto interact_ptr = std::make_shared<InteractOperation>(0.0,true, configuration,3.0);
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
