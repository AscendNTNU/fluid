#ifndef FLUID_FSM_CORE_H
#define FLUID_FSM_CORE_H

#include "status_publisher.h"
#include "operation/state_graph.h"
#include <memory>
#include <string>

namespace fluid {

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

		static bool auto_arm;		 										///< Determines whether the state machine
																			///< should auto arm or not.

		static bool auto_set_offboard;										///< Determines whether the state machine
																			///< should set itself into offboard
																			///< automatically or not.

		static double minX, minY, minZ, maxX, maxY, maxZ;					///< The bounding box restricting the
																			///< movement of the drone. When all
																			///< values are set to 0 the drone can
																			///< move freely.

		static double positionFollowHeight;									///< The height at which the drone will
																			///< follow a given target.	

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