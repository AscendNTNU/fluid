/**
 * @file interact_operation.h
 */

#ifndef INTERACT_OPERATION_H
#define INTERACT_OPERATION_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/DebugValue.h>

#include "operation.h"
#include "operation_identifier.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <std_msgs/Bool.h> //LAEiv
#include <std_msgs/Int16.h>

#include <ascend_msgs/SetInt.h>
#include <std_srvs/Trigger.h>

#include "mast.h"
#include "data_file.h"

#include <chrono>

/**
 * @brief Represents the operation where the drone is interact with the mast.
 */
class InteractOperation : public Operation {
   private:
	
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
    ros::Time approaching_t0;

    bool MAST_INTERACT;
    bool SHOW_PRINTS;
    bool GROUND_TRUTH;
    bool EKF;
    bool USE_PERCEPTION;
	InteractionState interaction_state = InteractionState::APPROACHING;
    uint8_t completion_count; //count the number of ticks since we completeted the current state

    std_msgs::Int16 number_fail;
    
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::Point desired_offset;
    
    ros::Subscriber ekf_module_pose_subscriber;
    ros::Subscriber ekf_state_vector_subscriber;
    ros::Subscriber module_pose_subscriber;
    ros::Subscriber gt_module_pose_subscriber;
    ros::Subscriber fh_state_subscriber;
    ros::Subscriber close_tracking_ready_subscriber;

    ros::ServiceClient start_close_tracking_client;
    ros::ServiceClient pause_close_tracking_client;    
    ros::ServiceServer close_tracking_lost_service;

    ros::Publisher interact_fail_pub;
    ros::Publisher altitude_and_yaw_pub;
    
    float MAX_ACCEL;
    float MAX_VEL;


    Mast mast;

    DataFile reference_state;
    DataFile drone_pose;
    DataFile gt_reference;



    
    /**
     * @brief Estimate the travel time from current position to OVER state
     * 
     * @return estimated mast to travel to the mast
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
    std::chrono::steady_clock::time_point close_tracking_ready_timeout;
    
    bool close_tracking_lost_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
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
    
   public:
    /**
     * @brief Sets up the subscriber for the module pose.
     * And specify at what yaw angle is the mast compare to the world frame.
     * 
     * @param mast_yaw yaw angle of the mast compare to the world frame.
     */
    explicit InteractOperation(const float& fixed_mast_yaw, const float& offset=3.0);

    /**
     * @brief Sets up max leaning angle to 4°, subscribe to mast pose topic,
     * Set up Publisher for attitude and its own position control,
     * Choose initial offset, set up transitino and init data_files.
     */
    void initialize() override;

    /**
     * @return true When the module has been extracted.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Makes sure the drone is following the module and reacting to the extraction signal.
     */
    void tick() override;
};

#endif
