#ifndef FLUID_FSM_CORE_H
#define FLUID_FSM_CORE_H

#include "status_publisher.h"
#include "operation/state_graph.h"
#include <memory>

namespace fluid {

	class Core {

	private:

		// Constructors are private as this is used as a singleton with members.

		Core() {};
		Core(Core const&) {};
		Core& operator=(Core const&) {};

		/**
		 * @return     The shared singleton instance of the graph.
		 */
		static fluid::StateGraph getGraph();

		/**
		 * 
		 * @return     The singleton instance of the status publisher.
		 */
		static fluid::StatusPublisher getStatusPublisher(); 

	public:

		static std::unique_ptr<fluid::StateGraph> graph_p_;					///< Interface for publishing 
																			///< status about the fsm.
		
		static std::unique_ptr<fluid::StatusPublisher> status_publisher_p_; ///< Provides the states which the
		                                                                 	///<  operation can consists
        	                                                 				///< of and how they are connected.
	};
}

#endif