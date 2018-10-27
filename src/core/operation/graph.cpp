//
//  graph.cpp
//  fluid_fsm
//
//  Created by Simen Gangstad on 09/10/2018.
//  Copyright © 2018 Simen Gangstad. All rights reserved.
//

#include "../../../include/core/operation/graph.h"
#include <iostream>
#include <list>
#include <iterator>
#include <algorithm>

fluid::Graph::Graph() {
    adjacency_list = std::make_unique<AdjacencyList>();

}

void fluid::Graph::addEdges(std::vector<Edge> const &edges) {
    for (auto edge: edges) {
        // If the state isn't added in the adjacency list
        if (adjacency_list->find(edge.source->identifier) == adjacency_list->end()) {
            std::vector<Node> neighbors;
            neighbors.push_back(edge.destination);
            adjacency_list->insert(std::make_pair(edge.source->identifier, neighbors));

            // As this state didn't exist in the graph, we add it the list of states
            states_.push_back(edge.source);
        }
        else {
            adjacency_list->at(edge.source->identifier).push_back(edge.destination);
        }
    }
}

std::vector<std::shared_ptr<fluid::State>> fluid::Graph::getStates() {
    return states_;
}

std::list<std::shared_ptr<fluid::State>> fluid::Graph::getPlanToEndState(std::string start_state_identifier,
                                                                         std::string end_state_identifier) {
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

        for(const auto neighborState : adjacency_list->at(start_state_identifier)) {

            if(!visited[neighborState->identifier]) {
                visited[neighborState->identifier] = true;
                queue.push_back(neighborState->identifier);
            }
        }
    }


    // Transform plan of state identifiers into a plan of states
    std::list<std::shared_ptr<fluid::State>> states_in_plan;

    for (auto identifier : plan) {
        // Get state with this identifier from the state vector
        auto iterator = find_if(getStates().begin(), getStates().end(), [&identifier](const std::string & obj) {
            return obj == identifier;
        });

        if (iterator != getStates().end()) {
            states_in_plan.push_back(*iterator);
        }
    }

    return states_in_plan;
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
