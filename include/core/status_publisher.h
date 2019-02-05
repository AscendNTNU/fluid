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
		
		ros::Publisher publisher_p_;										///< Used to publish messages.

	public:

		ascend_msgs::FluidFsmStatus status;									///< Represents the state of the FSM.


		/**
		 * @brief      Initializes the status publisher.
		 * 
		 * The ros publisher isn't initialized in the constructor as this class is used as a static variable
		 * and node handle pointers can't be initialized when this class will be. 
		 */
		StatusPublisher();

		/**
		 * @brief      Initializes the status publisher.
		 *
		 * @param[in]  node_handle_p       Used to intiailize the ros publisher.
		 * @param[in]  message_queue_size  Amount of messages in the buffer.
		 */
		void initialize(ros::NodeHandlePtr node_handle_p, const unsigned int message_queue_size);

		/**
		 * @brief      Publishes the status on the topic.
		 */
		void publish(); 

	};
}


#endif