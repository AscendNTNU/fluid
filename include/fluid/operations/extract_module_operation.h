/**
 * @file extract_module_operation.h
 */

#ifndef EXTRACT_MODULE_OPERATION_H
#define EXTRACT_MODULE_OPERATION_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
        uint8_t finished_bitmask;
    };

	ExtractionState extraction_state = ExtractionState::APPROACHING;
    float fixed_mast_yaw; //Should be given by perception and known before entering in ExtractModuleOperation
    const float speed = 500;
    
    mavros_msgs::PositionTarget module_state;
    mavros_msgs::PositionTarget previous_module_state;
    
    TransitionSetpointStruct transition_state;
    geometry_msgs::Point desired_offset;
    
    ros::Subscriber module_pose_subscriber;
    ros::Publisher attitude_pub;
    mavros_msgs::AttitudeTarget attitude_setpoint;
    

    void modulePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose);

    ros::ServiceClient backpropeller_client;

    bool called_backpropeller_service = false;

    geometry_msgs::Vector3 derivate(geometry_msgs::Vector3 actual, geometry_msgs::Vector3 last, ros::Time dt_ros);
    mavros_msgs::PositionTarget rotate(mavros_msgs::PositionTarget setpoint, float yaw=0);
    geometry_msgs::Vector3 rotate(geometry_msgs::Vector3 pt, float yaw=0);
    geometry_msgs::Point rotate(geometry_msgs::Point pt, float yaw=0);
    geometry_msgs::Vector3 estimateModuleVel();
    geometry_msgs::Vector3 estimateModuleAccel();

    //functions in realtion to the following of the mast
    //not sure if they should be public or private, but private seems nice, ikke sant?
    geometry_msgs::Quaternion accel_to_orientation(geometry_msgs::PointPtr accel);
    geometry_msgs::Quaternion euler_to_quaternion(double yaw, double roll, double pitch);
    void LQR_to_acceleration(mavros_msgs::PositionTarget* ref, geometry_msgs::PointPtr accel_targ, bool use_sqrt);
    void update_attitude_input(mavros_msgs::PositionTarget module,mavros_msgs::PositionTarget offset, bool use_sqrt);
    
   public:
    /**
     * @brief Sets up the subscriber for the module pose.
     * And specify at what yaw angle is the mast compare to the world frame.
     * 
     * @param mast_yaw yaw angle of the mast compare to the world frame.
     */
    explicit ExtractModuleOperation(float mast_yaw);

    /**
     * @brief Sets up #speed at which to move.
     */ //TODO: update comment
    void initialize() override;

    /**
     * @brief //create a header for the logfile.
     */
    void initLog();

    /**
     * @brief //Add a new line to the logfile with the actual position and velocity
     */
    void saveLog();

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
