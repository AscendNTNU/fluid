//
//  Created by Simen Gangstad on 09/10/2018.
//

#include <iostream>
#include <iterator>
#include <vector>

#include "graph.h"

fluid::Graph::Graph() {
    adjacency_list = std::make_unique<AdjacencyList>();
}

void fluid::Graph::addEdges(std::vector<fluid::Edge<Node>> edges) {
    for (auto edge : edges) {

        // If the node isn't added in the adjacency list, we add the the node and its neighbour (edge destination).
        if (adjacency_list->find(edge.source->identifier) == adjacency_list->end()) {

            std::vector<Node> neighbours;
            neighbours.push_back(edge.destination);
            adjacency_list->insert(std::make_pair(edge.source->identifier, neighbours));

            // As this node didn't exist in the graph, we add it the list of nodes
            nodes_.push_back(edge.source);
        }
        // Node exists in graph, just add the new neighbour
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
