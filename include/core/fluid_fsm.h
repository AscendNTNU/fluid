#ifndef FLUID_FSM_FLUID_FSM_H
#define FLUID_FSM_FLUID_FSM_H

#include "status_publisher.h"
#include "operation/state_graph.h"

namespace fluid {

	static StateGraph graph;                                    ///< Provides the states which the operation can
        	                                                        ///< consist of and how they are connected.
	
	static fluid::StatusPublisher status_publisher; 			///< Interface for publishing status about the fsm.
}

#endif