//
//  graph.cpp
//  fluid_fsm
//
//  Created by Simen Gangstad on 09/10/2018.
//  Copyright © 2018 Simen Gangstad. All rights reserved.
//

#include "../../include/operation/graph.h"
#include <iostream>
#include <list>
#include <iterator>

fluid::Graph::Graph() {
    adjacency_list = std::make_unique<AdjacencyList>();
}

void fluid::Graph::addEdges(std::vector<Edge> const &edges) {
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

std::vector<std::string> fluid::Graph::getPlanToEndState(std::string start_state_identifier, std::string end_state_identifier) {
    std::map<std::string, bool> visited;

    for (auto const& item : *adjacency_list) {
        visited[item.first] = false;
   }

    std::list<std::string> queue;
    visited[start_state_identifier] = true;
    queue.push_back(start_state_identifier);
    std::vector<std::string> plan;

    while (!queue.empty()) {

        start_state_identifier = queue.front();
        plan.push_back(start_state_identifier);
        queue.pop_front();

        if (start_state_identifier == end_state_identifier) {
            break;
        }

        for(auto neighborState : adjacency_list->at(start_state_identifier)) {

            if(!visited[neighborState->identifier]) {
                visited[neighborState->identifier] = true;
                queue.push_back(neighborState->identifier);
            }
        }
    }

    return plan;
}

void fluid::Graph::print() {
    for (auto const& item : *adjacency_list) {
        std::cout << "\n Adjacency list of vertex " << item.first << "\n head ";
        
        for (auto neighbor : item.second) {
            std::cout << "-> " << neighbor->identifier;
        }
        
        std::cout << "\n";
    }
}
