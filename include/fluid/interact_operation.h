/**
 * @file interact_operation.h
 */

#ifndef INTERACT_OPERATION_H
#define INTERACT_OPERATION_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/DebugValue.h>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include "mast.h"
#include "data_file.h"

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
        mavros_msgs::PositionTarget state;
        uint8_t finished_bitmask;
    };

    /**
     * @brief Determine when we enter the state APPROACHING
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr approaching_t0;
    
    rclcpp::Publisher<mavros_msgs::PositionTarget>::SharedPtr ekf_module_pose_subscriber;
    rclcpp::Publisher<mavros_msgs::DebugValue>::SharedPtr ekf_state_vector_subscriber;
    rclcpp::Publisher<geometry_msgs::PoseWithCovarianceStampedConstPtr>::SharedPtr module_pose_subscriber;
    rclcpp::Publisher<geometry_msgs::PoseWithCovarianceStampedConstPtr>::SharedPtr gt_module_pose_subscriber;
    rclcpp::Publisher<td_msgs::Bool>::SharedPtr fh_state_subscriber;
    rclcpp::Publisher<td_msgs::Bool>::SharedPtr close_tracking_ready_subscriber;

    ros::ServiceClient start_close_tracking_client;
    ros::ServiceClient pause_close_tracking_client;    

    rclcpp::Publisher<std_msgs::Int16>::SharedPtr interact_fail_pub;
    rclcpp::Publisher<mavros_msgs::PositionTarget>::SharedPtr altitude_and_yaw_pub;

    

    bool SHOW_PRINTS;
    bool GROUND_TRUTH;
    bool EKF;
    bool USE_PERCEPTION;
	InteractionState interaction_state = InteractionState::APPROACHING;
    uint8_t completion_count; //count the number of ticks since we completeted the current state

    const bool steady;
    bool autoPublish;

    std_msgs::Int16 number_fail;
    
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::Point desired_offset;
    

    
    float MAX_ACCEL;
    float MAX_VEL;

    Mast mast;

    DataFile reference_state;
    DataFile drone_pose;
    DataFile gt_reference;



    
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
    
    void ekfStateVectorCallback(const mavros_msgs::DebugValue ekf_state);
    void ekfModulePoseCallback(const mavros_msgs::PositionTarget module_state);
    void gt_modulePoseCallback(const geometry_msgs::PoseStamped module_pose);
    void gt_modulePoseCallbackWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose);
    void gt_modulePoseCallbackWithoutCov(const geometry_msgs::PoseStampedConstPtr module_pose);
    void FaceHuggerCallback(const std_msgs::Bool released);
    void closeTrackingCallback(std_msgs::Bool ready);
    void finishInteraction();
    bool faceHugger_is_set;     // true as soon av facehugger is released from drone
    
    mavros_msgs::PositionTarget rotate(mavros_msgs::PositionTarget setpoint, float yaw);
    geometry_msgs::Vector3 estimateModuleVel();
    geometry_msgs::Vector3 estimateModuleAccel();

    void update_transition_state();

    ros::Subscriber pose_subscriber;
    geometry_msgs::PoseStamped current_pose;
    void poseCallback(const nav_msgs::OdometryConstPtr pose);

    ros::Subscriber twist_subscriber;
    geometry_msgs::TwistStamped current_twist;
    void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);
    geometry_msgs::Vector3 current_accel;
    int rate_int;

    ros::Publisher setpoint_publisher;
    ros::NodeHandle node_handle;
    mavros_msgs::PositionTarget setpoint;

    void publishSetpoint();
    virtual bool hasFinishedExecution() const = 0;
    virtual void initialize() {}
    virtual void tick() {}
    geometry_msgs::PoseStamped getCurrentPose() const;
    geometry_msgs::TwistStamped getCurrentTwist() const;
    geometry_msgs::Vector3 getCurrentAccel() const;
    float getCurrentYaw() const;
    geometry_msgs::Vector3 orientation_to_acceleration(geometry_msgs::Quaternion orientation);

   public:
    explicit InteractOperation(const float& fixed_mast_yaw, const float& offset=3.0, const bool& steady,
    const bool& autoPublish, MastNodeConfiguration config);
    void initialize() override;
    bool hasFinishedExecution() const override;
    void tick() override;
    virtual void perform(std::function<bool(void)> should_tick, bool should_halt_if_steady);

};

#endif
