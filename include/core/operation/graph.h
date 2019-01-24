//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_GRAPH_H
#define FLUID_FSM_GRAPH_H

#include <memory>
#include <vector>
#include <map>
#include "../state.h"
#include "edge.h"
#include "../identifiable.h"

namespace fluid {

    typedef std::shared_ptr<fluid::Identifiable> Node;
    typedef std::map<std::string, std::vector<Node>> AdjacencyList;

    /** \class Graph
     * 
     *  \brief Represents a graph of nodes.
     */
    class Graph {
    protected:

        std::unique_ptr<AdjacencyList> adjacency_list;              ///< Vector containing all the 
                                                                    ///< connections in the graph

        std::vector<Node> nodes_;                                   ///< Vector of all the nodes in the graph
        
    public:

        /**
         * Initializes the graph.
         */
        Graph();

        /**
         * Adds edges to the graph, these edges form the connections between the nodes in the graph.
         *
         * @param edges Vector of edges.
         */
        void addEdges(std::vector<fluid::Edge<Node>> edges);

        /**
         * @return The nodes in the graph.
         */
        std::vector<Node> getNodes();

        /**
         * Prints the graph.
         */
        void print();
    };
}

#endif /* FLUID_FSM_GRAPH_H */
