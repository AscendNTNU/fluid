//
//  Created by Simen Gangstad on 24/01/2019.
//

#ifndef FLUID_FSM_STATE_GRAPH_H
#define FLUID_FSM_STATE_GRAPH_H

#include <memory>
#include <string>

#include "state.h"

namespace fluid {

    /** \class Edge
     *  \brief Represents an edge/connection between two nodes a the graph.
     */
    template <class T> class Edge {
    public:
        
        const T source;                 ///< The source of the edge
        const T destination;            ///< The desination of the edge
        
        /** Initializes the edge with its respective source and destination.
         * 
         */
        Edge(T source, T destination): source(source), destination(destination) {}
    };


    typedef std::map<std::string, std::vector<std::shared_ptr<State>>> AdjacencyList;

    /** \class StateGraph
     * 
     *  \brief Represents a graph with states.
     */
    class StateGraph {

        std::unique_ptr<AdjacencyList> adjacency_list_ptr_;         ///< Vector containing all the 
                                                                    ///< connections in the graph

        std::vector<std::shared_ptr<State>> states_;                                 ///< Vector of all the states in the graph
        

    public:

        std::shared_ptr<fluid::State> current_state_ptr;              ///< The current state of the state graph.

        /**
         * @brief      Sets up the graph with the respective states.
         */
        StateGraph();

        /**
         * Adds edges to the graph, these edges form the connections between the states in the graph.
         *
         * @param edges Vector of edges.
         */
        void addEdges(std::vector<fluid::Edge<std::shared_ptr<State>>> edges);

        /**
         * @return The states in the graph.
         */
        std::vector<std::shared_ptr<State>> getStates();

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
         * @return A flag determining whether two states are connected. 
         */
        bool areConnected(std::string start_state_identifier, std::string end_state_identifier);

        /**
         * @return The state with the given identifier in the graph. Will return a nullptr if not found.
         */
        std::shared_ptr<fluid::State> getStateWithIdentifier(std::string identifier);

        /**
         * Returns a stream with information about this state graph.
         */
        friend std::ostream& operator<<(std::ostream& ostream, const fluid::StateGraph& state_graph);
    };
}

#endif /* FLUID_FSM_STATE_GRAPH_H */
