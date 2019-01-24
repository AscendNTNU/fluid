//
//  Created by Simen Gangstad on 09/10/2018.
//

#include "../../../include/core/operation/graph.h"
#include "../../../include/core/operation/edge.h"

#include <iostream>
#include <iterator>
#include <vector>

fluid::Graph::Graph() {
    adjacency_list = std::make_unique<AdjacencyList>();
}

void fluid::Graph::addEdges(std::vector<fluid::Edge<Node>> edges) {
    for (auto edge : edges) {
        // If the state isn't added in the adjacency list
        if (adjacency_list->find(edge.source->identifier) == adjacency_list->end()) {
            std::vector<Node> neighbors;
            neighbors.push_back(edge.destination);
            adjacency_list->insert(std::make_pair(edge.source->identifier, neighbors));

            // As this state didn't exist in the graph, we add it the list of states
            nodes_.push_back(edge.source);
        }
        else {
            adjacency_list->at(edge.source->identifier).push_back(edge.destination);
        }
    }
}

std::vector<fluid::Node> fluid::Graph::getNodes() {
    return nodes_;
}

void fluid::Graph::print() {
    for (auto const& item : *adjacency_list) {
        
        std::cout << "\n Adjacency list of vertex " << item.first << "\n head ";
        
        for (auto &neighbor : item.second) {
            std::cout << "-> " << neighbor->identifier;
        }
        
        std::cout << "\n";
    }
}
