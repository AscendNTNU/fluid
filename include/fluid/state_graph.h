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
        
        const T source;                 
        const T destination;            
        
        Edge(T source, T destination): source(source), destination(destination) {}
    };


    typedef std::map<StateIdentifier, std::vector<std::shared_ptr<State>>> AdjacencyList;

    class StateGraph {

        std::unique_ptr<AdjacencyList> adjacency_list_ptr;

        std::vector<std::shared_ptr<State>> states;    
        

    public:

        std::shared_ptr<fluid::State> current_state_ptr; 

        StateGraph();

        void addEdges(const std::vector<fluid::Edge<std::shared_ptr<State>>>& edges);

        std::vector<std::shared_ptr<State>> getStates() const;

        /**
         * Checks if the start state and the end state are connected using a breadth first search and
         * returns the shortest path between them.
         */
        std::vector<std::shared_ptr<fluid::State>> getPathToEndState(const StateIdentifier& start_state_identifier,
                                                                     const StateIdentifier& end_state_identifier, 
                                                                     const bool& should_include_move) const;

        bool areConnected(const StateIdentifier& start_state_identifier, const StateIdentifier& end_state_identifier) const;

        /**
         * @return The state with the given identifier in the graph. **Will return a nullptr if not found**.
         */
        std::shared_ptr<fluid::State> getStateWithIdentifier(const StateIdentifier& identifier) const;
    };
}

#endif 
