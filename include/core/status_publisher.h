// Created by simengangstad 31.02.19

#ifndef FLUID_FSM_STATUS_PUBLISHER_H
#define FLUID_FSM_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <string>
#include <ascend_msgs/FluidFsmStatus.h>
#include <memory>

namespace fluid {

	/**
	 */
	class StatusPublisher {

	private:
		
		ros::NodeHandle node_handle_;										///< Used to set up the publisher.

		ros::Publisher publisher_p_;										///< Used to publish messages.

	public:

		ascend_msgs::FluidFsmStatus status;									///< Represents the state of the FSM.


		/**
		 * @brief      Initializes the status publisher.
		 * 
		 * Sets up the pose publisher.
		 * 
		 * @param[in]  message_queue_size  Amount of messages in the buffer.
		 */
		StatusPublisher(const unsigned int message_queue_size);

		/**
		 * @brief      Publishes the status on the topic.
		 */
		void publish(); 

	};
}


#endif