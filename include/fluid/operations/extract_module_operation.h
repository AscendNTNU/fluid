/**
 * @file extract_module_operation.h
 */

#ifndef EXTRACT_MODULE_OPERATION_H
#define EXTRACT_MODULE_OPERATION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "operation.h"
#include "operation_identifier.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"

/**
 * @brief Represents the operation where the drone is extracting the module.
 */
class ExtractModuleOperation : public Operation {
   private:
	
	enum class ExtractionState {
		APPROACHING,
		OVER,
		BEHIND_WITH_HOOKS,
	 	EXTRACTING,
		EXTRACTED
	};

    struct TransitionSetpointStruct {
        float max_vel;
        float cte_acc;
        mavros_msgs::PositionTarget state;
        uint8_t finished_bitmask; //updated but unused
    };

	ExtractionState extraction_state = ExtractionState::APPROACHING;
    uint8_t completion_count; //count the number of ticks since we completeted the current state
    float fixed_mast_yaw; //Should be given by perception and known before entering in ExtractModuleOperation
    
    mavros_msgs::PositionTarget module_state;
    mavros_msgs::PositionTarget previous_module_state;
    
    /**
     * @brief the pitch, roll and trigonometric angle of the mast
     */
    geometry_msgs::Vector3 mast_angle;
    
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::Point desired_offset;
    
    ros::Subscriber module_pose_subscriber;
    ros::Publisher attitude_pub;
    ros::Publisher altitude_and_yaw_pub;
    mavros_msgs::AttitudeTarget attitude_setpoint;
    geometry_msgs::Vector3 accel_target;
    

    void modulePoseCallback(const geometry_msgs::PoseStampedConstPtr module_pose);

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
    explicit ExtractModuleOperation(const float& fixed_mast_yaw);

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
     * @brief //Add a new line to the logfile with the actual position and velocity
     */
    void saveLog(const std::string file_name, const mavros_msgs::PositionTarget data);

    /**
     * @brief //Add a new line to the logfile with the actual position and velocity
     */
    void saveLog(const std::string file_name, const geometry_msgs::PoseStamped pose, const geometry_msgs::TwistStamped vel, const geometry_msgs::Vector3 accel);

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
