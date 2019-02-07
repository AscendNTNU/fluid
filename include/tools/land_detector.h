//
// Created by simengangstad on 22.11.18.
//

#ifndef FLUID_FSM_LAND_DETECTOR_H
#define FLUID_FSM_LAND_DETECTOR_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

namespace fluid {

    /**
     * @brief Uses velocity and position to figure out when the drone has landed.
     * 
     * The land detector will check if the current position is near the land position and if
     * the current velocity in z-direction (downwards) is close to 0. To achieve this, this class
     * will subscribe to mavros topics and check against the setpoints.  
     */
    class LandDetector {
	private:

        ros::NodeHandle node_handle_;                           ///< Used to initialize the subscribers;

		ros::Subscriber pose_subscriber_;						///< Provides the land detector with a stream of the 
																///< current position.


		geometry_msgs::PoseStamped current_pose_;				///< Most recent pose received from the subscriber
																///< callback.

	
        /**
         * Gets fired when mavros publishes a pose on the topic "mavros/local_position/pose".
         */
        void poseCallback(const geometry_msgs::PoseStampedPtr pose_p);


		ros::Subscriber velocity_subscriber_; 					///< Provides the land detector with a stream of the
																///< current velocity 

		geometry_msgs::TwistStamped current_velocity_;			///< Most recent velocity received from the subscriber
																///< callback.

        /**
         * Gets fired when mavros publishes a velocity on the topic "mavros/local_position/velocity".
         */
        void velocityCallback(const geometry_msgs::TwistStampedPtr velocity_p);

	public:
		/**
		 * @brief      Initializes the land detector and sets up the necessary subscriptions.
		 *
		 * @param[in]  land_position  The position the drone should land at.
		 */
    	LandDetector() : 
    	pose_subscriber_(node_handle_.subscribe("mavros/local_position/pose", 
    											1000, 
    											&LandDetector::poseCallback, 
    											this)),
    	velocity_subscriber_(node_handle_.subscribe("mavros/local_position/velocity",
    												1000, 
    												&LandDetector::velocityCallback,
    												this)) {}

        /**
         * @brief      Determines if the drone has landed by checking the current position and the current
         *              velocity.
         *
         * @param[in]  land_position  The target land position the drone should land at.
         *
         * @return     True if has landed, False otherwise.
         */
    	bool hasLanded(mavros_msgs::PositionTarget land_position);
    };
}    

#endif //FLUID_FSM_LAND_DETECTOR_H
