//
//  graph.cpp
//  fluid_fsm
//
//  Created by Simen Gangstad on 09/10/2018.
//  Copyright © 2018 Simen Gangstad. All rights reserved.
//

#include "graph.h"
#include <iostream>

Graph::Graph(std::vector<Edge> const &edges) {
    adjacency_list = std::make_unique<AdjacencyList>();
    
    for (auto edge: edges) {
        if (adjacency_list->find(edge.source->identifier) == adjacency_list->end()) {
            std::vector<Node> neighbors;
            neighbors.push_back(edge.destination);
            adjacency_list->insert(std::make_pair(edge.source->identifier, neighbors));
        }
        else {
            adjacency_list->at(edge.source->identifier).push_back(edge.destination);
        }
    }
}

void Graph::print() {
    for (auto const& item : *adjacency_list) {
        std::cout << "\n Adjacency list of vertex " << item.first << "\n head ";
        
        for (auto neighbor : item.second) {
            std::cout << "-> " << neighbor->identifier;
        }
        
        std::cout << "\n";
    }
}
