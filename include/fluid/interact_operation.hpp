/**
 * @file interact_operation.h
 */

#ifndef INTERACT_OPERATION_H
#define INTERACT_OPERATION_H

#include <chrono>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/debug_value.hpp>

#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fluid/mast.hpp"
//#include "fluid/data_file.hpp"

struct MastNodeConfiguration {
    /**
     * @brief whether ekf is used or not
     */
    const bool ekf;

    /**
     * @brief whether use_perception is used or not
     */
    const bool use_perception;

    /**
     * @brief The unified refresh rate across the operation machine.
     */
    const int refresh_rate;

    /**
     * @brief Whether the drone will arm automatically.
     */
    const bool should_auto_arm;

    /**
     * @brief Whether the drone will go into offboard mode automatically.
     */
    const bool should_auto_offboard;

    /**
     * @brief Specifies the radius for within we can say that the drone is at a given position.
     */
    const float distance_completion_threshold;

    /**
     * @brief Specifies how low the velocity has to be before we issue that a given operation has completed. This
     *        serves the purpose to prevent osciallations in the case when new operations are fired the moment the
     *        previous completes. With this threshold, we have to wait for the drone to be "steady" at the current
     *        position before moving on.
     */
    const float velocity_completion_threshold;

    /**
     * @brief Height used when e.g. 0 was given for a setpoint.
     */
    const float default_height;

    /**
     * @brief Show some debugging prints during the interact operation
     */
    const bool interaction_show_prints;
    
    /**
     * @brief Show some debugging prints during the interact operation
     */
    const float interact_max_vel;
    
    /**
     * @brief Use ground_truth data for interact operation.
     */
    const float interact_max_acc;  

    /**
     * @brief max angle ardupilot parameter for the travel operation.
     */
    const float travel_max_angle;  

    /**
     * @brief 3D offset of the Face_hugger compared to the drone center
     */
    const float* fh_offset;

    /**
     * @brief max speed ardupilot parameter for the travel operation.
     */
    const float travel_speed;  

    /**
     * @brief max accel ardupilot parameter for the travel operation.
     */
    const float travel_accel;  
};

/**
 * @brief Represents the operation where the drone is interact with the mast.
 */
class InteractOperation : public rclcpp::Node {
   private:
	MastNodeConfiguration config;
	enum class InteractionState {
		APPROACHING,
        READY,
		OVER,
		INTERACT,
        EXIT,
		EXTRACTED
	};

    struct TransitionSetpointStruct {
        float max_vel;
        float cte_acc;
        mavros_msgs::msg::PositionTarget state;
        uint8_t finished_bitmask;
    };

    /**
     * @brief Determine when we enter the state APPROACHING
     */
    rclcpp::Time approaching_t0;
    
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr ekf_module_pose_subscriber;
    rclcpp::Subscription<mavros_msgs::msg::DebugValue>::SharedPtr ekf_state_vector_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr module_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gt_module_pose_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fh_state_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr close_tracking_ready_subscriber;
    
    //rclcpp::Client<ascend_msgs::msg::SetInt>::SharedPtr start_close_tracking_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_close_tracking_client;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr interact_fail_pub;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr altitude_and_yaw_pub;

    

    bool SHOW_PRINTS;
    bool GROUND_TRUTH;
    bool EKF;
    bool USE_PERCEPTION;
	InteractionState interaction_state = InteractionState::APPROACHING;
    uint8_t completion_count; //count the number of ticks since we completeted the current state
    
    bool faceHugger_is_set;
    bool rcvd_module_pose;

    std_msgs::msg::Int16 number_fail;
    
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::msg::Point desired_offset;
    

    
    float MAX_ACCEL;
    float MAX_VEL;

    Mast mast;

    //DataFile reference_state;
    //DataFile drone_pose;
    //DataFile gt_reference;



    
    /**
     * @brief Estimate the travel time from current position to OVER state
     * 
     * @return estimated time to travel to the mast
     */
    float estimate_time_to_mast();


    /**
     * @brief state whether a service call to switch close tracking has been made or not.
     * 
     */
    bool close_tracking_is_set;
    
    /**
     * @brief state whether close tracking is activated or not
     */
    bool close_tracking_is_ready;
    
    void ekfStateVectorCallback(const mavros_msgs::msg::DebugValue ekf_state);
    void ekfModulePoseCallback(const mavros_msgs::msg::PositionTarget::SharedPtr module_state);
    void gt_modulePoseCallback(const geometry_msgs::msg::PoseStamped module_pose);
    void gt_modulePoseCallbackWithCov(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr module_pose);
    void gt_modulePoseCallbackWithoutCov(geometry_msgs::msg::PoseStamped::SharedPtr module_pose_ptr);
    void FaceHuggerCallback(const std_msgs::msg::Bool::SharedPtr released);
    void closeTrackingCallback(std_msgs::msg::Bool::SharedPtr ready);
    

    // void gt_modulePoseCallbackWithoutCRCLCPP_INFOsoon as we have received the first interactio pt state
    
    mavros_msgs::msg::PositionTarget rotate(mavros_msgs::msg::PositionTarget setpoint, float yaw);
    geometry_msgs::msg::Vector3 estimateModuleVel();
    geometry_msgs::msg::Vector3 estimateModuleAccel();

    void update_transition_state();

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber;
    geometry_msgs::msg::PoseStamped current_pose;
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber;
    geometry_msgs::msg::TwistStamped current_twist;
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist);
    geometry_msgs::msg::Vector3 current_accel;
    int rate_int;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_publisher;
    mavros_msgs::msg::PositionTarget setpoint;

    //rclcpp::Publisher<CommandWithParameters>::SharedPtr setpoint_publisher;
    //CommandWithParameters setpoint;

    
    geometry_msgs::msg::PoseStamped getCurrentPose() const;
    geometry_msgs::msg::TwistStamped getCurrentTwist() const;
    geometry_msgs::msg::Vector3 getCurrentAccel() const;
    float getCurrentYaw() const;
    geometry_msgs::msg::Vector3 orientation_to_acceleration(geometry_msgs::msg::Quaternion orientation);

   public:
    explicit InteractOperation(const float& fixed_mast_yaw,
        const bool& autoPublish, MastNodeConfiguration config, const float& offset=3.0);
    void initialize();
    bool hasFinishedExecution() const;
    void tick();
    void perform(std::function<bool(void)> should_tick, bool should_halt_if_steady);
    //Made public to comply with the way fluid was run.
    void publishSetpoint();
    bool autoPublish;

};

#endif
