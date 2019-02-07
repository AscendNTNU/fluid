#ifndef FLUID_FSM_FLUID_FSM_H
#define FLUID_FSM_FLUID_FSM_H

#include "status_publisher.h"
#include "operation/state_graph.h"

namespace fluid {

	// TODO: Can remove node handle, just initialize it every time
	// TODO: Use static singleton pattern

	extern StateGraph graph;                                 ///< Provides the states which the operation can
        	                                                 ///< consist of and how they are connected.
	
	extern fluid::StatusPublisher status_publisher; 		 ///< Interface for publishing status about the fsm.
}

#endif