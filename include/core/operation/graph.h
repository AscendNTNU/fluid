//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_GRAPH_H
#define FLUID_FSM_GRAPH_H

#include <memory>
#include <vector>
#include <map>
#include <set>
#include "../state.h"
#include "edge.h"
#include "../../states/state_identifier.h"

namespace fluid {
    typedef std::shared_ptr<State> Node;
    typedef std::map<StateIdentifier, std::vector<Node>> AdjacencyList;

    /** \class Graph
     *  \brief Represents the graph of states and transitions.
     */
    class Graph {
    private:

        std::unique_ptr<AdjacencyList> adjacency_list;         ///< Vector containing all the connections in the graph

        std::vector<std::shared_ptr<State>> states_;           ///< Vector of all the states in the state graph

    public:

        /**
         * Initializes the graph.
         */
        Graph();

        /**
         * Adds edges to the graph, these edges form the connections between the nodes in the graph. In other words
         * these edges specify which state changes are allowed.
         *
         * @param edges Vector of edges.
         */
        void addEdges(std::vector<Edge> const &edges);

        /**
         * @return The states in the state graph.
         */
        std::vector<std::shared_ptr<fluid::State>> getStates();

        /**
         * Finds the plan for going from the specified start state to the specified end state (bredth-first search).
         *
         * @param start_state_identifier The identifier of the start state we begin at.
         * @param end_state_identifier The identifier of the end state we want to transition to.
         * @return Queue of states one has to transition to in order to get to the final state.
         */
        std::list<std::shared_ptr<fluid::State>> getPlanToEndState(fluid::StateIdentifier start_state_identifier, fluid::StateIdentifier end_state_identifier);

        /**
         * Prints the graph.
         */
        void print();
    };
}

#endif /* FLUID_FSM_GRAPH_H */
