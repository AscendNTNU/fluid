//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_GRAPH_H
#define FLUID_FSM_GRAPH_H

#include <memory>
#include <vector>
#include <map>
#include <string>
#include "state.h"
#include "edge.h"

/** \class Graph
 *  \brief Represents the graph of states and transitions.
 */
class Graph {
public:
    
    typedef std::shared_ptr<State> Node;
    typedef std::map<std::string, std::vector<Node>> AdjacencyList;
    
    std::unique_ptr<AdjacencyList> adjacency_list; ///< Vector containing all the connections in the graph
    
    /** Initializes the graph with a vector of edges.
     *  These edges for the connetions between the states in the graph.
     */
    Graph(std::vector<Edge> const &edges);

    
    /**
     * Prints the graph.
     */
    void print();
};

#endif /* FLUID_FSM_GRAPH_H */
