/**
 * @file interact_operation.h
 */

#ifndef INTERACT_OPERATION_H
#define INTERACT_OPERATION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "operation.h"
#include "operation_identifier.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "std_msgs/Bool.h" //LAEiv

#include "mast.h"

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
    ros::Time startApproaching;

    bool SHOW_PRINTS;
    bool GROUND_TRUTH;
	InteractionState interaction_state = InteractionState::APPROACHING;
    uint8_t completion_count; //count the number of ticks since we completeted the current state
    
    
    mavros_msgs::PositionTarget module_state;
    mavros_msgs::PositionTarget previous_module_state;
    
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::Point desired_offset;
    
    ros::Subscriber module_pose_subscriber;
    ros::Subscriber fh_state_subscriber; //LAEiv
    ros::Publisher attitude_pub;
    ros::Publisher altitude_and_yaw_pub;
    mavros_msgs::AttitudeTarget attitude_setpoint;
    geometry_msgs::Vector3 accel_target;
    
    float LQR_gains[4];
    float K_LQR_X[2];
    float K_LQR_Y[2];
    float MAX_ACCEL;
    float MAX_VEL;

    Mast mast;
    
    /**
     * @brief Estimation of the time the drone will take to got from
     * the Approach state to the interact state
     */
    float estimate_time_to_mast;
    
    void modulePoseCallback(const geometry_msgs::PoseStampedConstPtr module_pose);
    void FaceHuggerCallback(const std_msgs::Bool released);
    void finishInteraction();
    bool facehugger_released;   // true as soon av facehugger is released from drone
    bool faceHugger_is_set;     // true after facehugger released and interact-cycle completed
    
    ros::ServiceClient backpropeller_client;

    bool called_backpropeller_service = false;

    mavros_msgs::PositionTarget rotate(mavros_msgs::PositionTarget setpoint, float yaw=0);
    geometry_msgs::Vector3 rotate(geometry_msgs::Vector3 pt, float yaw=0);
    geometry_msgs::Point rotate(geometry_msgs::Point pt, float yaw=0);
    geometry_msgs::Vector3 estimateModuleVel();
    geometry_msgs::Vector3 estimateModuleAccel();

    
    
    //functions in relation to the following of the mast
    //not sure if they should be public or private
    geometry_msgs::Quaternion accel_to_orientation(geometry_msgs::Vector3 accel);
    geometry_msgs::Vector3 LQR_to_acceleration(mavros_msgs::PositionTarget ref);
    void update_attitude_input(mavros_msgs::PositionTarget offset);

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
     * @brief //create a header for the logfile.
     */
    void initLog(const std::string file_name);

    /**
     * @brief //Add a new line to the logfile with the given position, velocity and acceleration
     */
    void saveLog(const std::string file_name, const mavros_msgs::PositionTarget data);

    /**
     * @brief //Add a new line to the logfile with the given position, velocity and acceleration
     */
    void saveLog(const std::string file_name, const geometry_msgs::PoseStamped pose, const geometry_msgs::TwistStamped vel, const geometry_msgs::Vector3 accel);

    /**
     * @brief //create a header for the setpoint logfile. 
     *          It only includes acceleration setpoints
     */
    void initSetpointLog(const std::string file_name);

    /**
     * @brief //Add a new line to the logfile with acceleration setpoint
     */
    void saveSetpointLog(const std::string file_name, const geometry_msgs::Vector3 accel);


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
