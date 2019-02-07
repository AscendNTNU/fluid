#ifndef FLUID_FSM_CORE_H
#define FLUID_FSM_CORE_H

#include "status_publisher.h"
#include "operation/state_graph.h"
#include <memory>

namespace fluid {

	/**
	 * @brief      Describes a boundry box, a closed area from -x to x, -y to y and -z to z. Within the FSM a 
	 * 			   boundry of (0, 0, 0) will describe no boundry, in other words, the drone can move everywhere.
	 */
	struct Boundry {

		/**
		 * The values of the boundry.
		 */
		double x, y, z;
	};

	class Core {

	private:

		// Constructors are private as this class is used as a singleton with members.

		Core() {};
		Core(Core const&) {};
		Core& operator=(Core const&) {};

		static std::shared_ptr<fluid::StateGraph> graph_p_;					///< Interface for publishing 
																			///< status about the fsm.
		
		static std::shared_ptr<fluid::StatusPublisher> status_publisher_p_; ///< Provides the states which the
		                                                                 	///< operation can consists
        	                                                 				///< of and how they are connected.
	public: 

		static unsigned int refresh_rate;									///< The unified refresh rate across 
																			///< the state machine.
																			///< Specifies how fast the operation
																			///< server and its underlying servies 
																			///< will run. E.g. a refresh rate of 10
																			///< hertz will make the states and
																			///< transitions execute their logic every 
																			///< 100 ms (in best case).

		static unsigned int message_queue_size;								///< The unified message queue size for 
																			///< the different ros components within
																			///< the FSM.


		static fluid::Boundry boundry;

		/**
		 * @return     The shared singleton instance of the graph.
		 */
		static std::shared_ptr<fluid::StateGraph> getGraphPtr();

		/**
		 * 
		 * @return     The singleton instance of the status publisher.
		 */
		static std::shared_ptr<fluid::StatusPublisher> getStatusPublisherPtr(); 
	};
}

#endif