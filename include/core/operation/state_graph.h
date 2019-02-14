//
//  Created by Simen Gangstad on 24/01/2019.
//

#ifndef FLUID_FSM_STATE_GRAPH_H
#define FLUID_FSM_STATE_GRAPH_H

#include "../state.h"
#include "graph.h"
#include <memory>
#include <vector>

namespace fluid {

    /** \class StateGraph
     * 
     *  \brief Represents a graph with states.
     */
    class StateGraph: public fluid::Graph {

    public:

        std::shared_ptr<fluid::State> current_state_p;            ///< The current state of the state graph.

        /**
         * @brief      Sets up the graph with the respective states.
         */
        StateGraph();

        /**
         * Checks if the start state and the end state is connected using a breadth first search and
         * returns the shortest path between them.
         *
         * @param start_state_identifier The identifier of the start state we begin at.
         * @param end_state_identifier The identifier of the end state we want to transition to.
         * 
         * @return Vector of states one has to transition to in order to get to the end state.
         */
        std::vector<std::shared_ptr<fluid::State>> getPathToEndState(std::string start_state_identifier,
                                                                     std::string end_state_identifier);

        /**
         * @return The state with the given identifier in the graph. Will return a nullptr if not found.
         */
        std::shared_ptr<fluid::State> getStateWithIdentifier(std::string identifier);
    };
}

#endif /* FLUID_FSM_STATE_GRAPH_H */
