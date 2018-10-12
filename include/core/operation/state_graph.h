//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_STATE_GRAPH_H
#define FLUID_FSM_STATE_GRAPH_H

#include "graph.h"
#include "../states/hold_state.h"
#include "../states/idle_state.h"
#include "../states/land_state.h"
#include "../states/move_state.h"
#include "../states/take_off_state.h"

namespace fluid {
    /** \class StateGraph
     *  \brief Represents a graph with all of the states in fluid_fsm
     */
    class StateGraph: public Graph {
    public:
        
        std::shared_ptr<State> current_state; ///< The current state of the state graph
        
        /**
         * Initializes the state graph with a range of states.
         */
        StateGraph();
    };
}

#endif //FLUID_FSM_STATE_GRAPH_H
