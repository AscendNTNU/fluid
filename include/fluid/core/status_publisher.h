// Created by simengangstad 31.02.19

#ifndef FLUID_FSM_STATUS_PUBLISHER_H
#define FLUID_FSM_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <string>
#include <ascend_msgs/FluidFsmStatus.h>
#include <memory>

namespace fluid {

	class StatusPublisher {

	private:
		
		ros::NodeHandle node_handle_;										///< Used to set up the publisher.

		ros::Publisher publisher_p_;										///< Used to publish messages.

	public:

		ascend_msgs::FluidFsmStatus status;									///< Represents the state of the FSM.


		/**
		 * @brief      The status publisher is not set up here as it is used as a static variable and the
		 * 			   ros node handle will complain if it's initialized before ros::init. 
		 */
		StatusPublisher();

		/**
		 * @brief      Publishes the status on the topic.
		 */
		void publish(); 

	};
}


#endif