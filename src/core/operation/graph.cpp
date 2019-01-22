//
//  graph.cpp
//  fluid_fsm
//
//  Created by Simen Gangstad on 09/10/2018.
//  Copyright © 2018 Simen Gangstad. All rights reserved.
//

#include "../../../include/core/operation/graph.h"
#include "../../../include/core/operation/edge.h"

#include "../../../include/states/init_state.h"
#include "../../../include/states/idle_state.h"
#include "../../../include/states/take_off_state.h"
#include "../../../include/states/land_state.h"
#include "../../../include/states/hold_state.h"
#include "../../../include/states/move_state.h"

#include <memory>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <climits>

fluid::Graph::Graph() {
    adjacency_list = std::make_unique<AdjacencyList>();

}

void fluid::Graph::addEdges(std::vector<fluid::Edge> const &edges) {
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


std::vector<std::shared_ptr<fluid::State>> fluid::Graph::getPlanToEndState(std::string start_state_identifier,
                                                                           std::string end_state_identifier) {
    
    // BFS
    std::list<std::string> queue;
    std::map<std::string, bool> visited;
    std::map<std::string, std::string> pred;

    for (auto const& item : *adjacency_list) {
        visited[item.first] = false;
        pred[item.first] = "no_val";
    }

    visited[start_state_identifier] = true;
    queue.push_back(start_state_identifier);
    
    std::vector<std::string> plan;

    bool searching = true;

    while (!queue.empty() && searching) {

        start_state_identifier = queue.front();
        queue.pop_front();

        for(const auto &neighborState : adjacency_list->at(start_state_identifier)) {
            if(!visited[neighborState->identifier]) {
                visited[neighborState->identifier] = true;
                pred[neighborState->identifier] = start_state_identifier;

                queue.push_back(neighborState->identifier);

                if (neighborState->identifier == end_state_identifier) {
                    searching = false;

                    break;
                }
            }
        }
    }

    std::vector<std::string> path;
    std::string crawl = end_state_identifier;

    path.push_back(end_state_identifier);

    while (pred[crawl] != "no_val") {
        path.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    // Transform plan of state identifiers into a plan of states
    std::vector<std::shared_ptr<fluid::State>> states_in_plan;

    for (const auto &identifier : path) {
        states_in_plan.push_back(getStateWithIdentifier(identifier));
    }
    
    std::reverse(states_in_plan.begin(), states_in_plan.end());

    return states_in_plan;
}

std::shared_ptr<fluid::State> fluid::Graph::getStateWithIdentifier(std::string identifier) {
    // Get state with this identifier from the state vector
    for (auto state : getStates()) {
        if (state->identifier == identifier) {
            return state;
        }
    }

    return nullptr;
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

bool fluid::Graph::isInitialized() {
    return initialized_;
}

void fluid::Graph::initialize(unsigned int refresh_rate) {

    node_handle_p = ros::NodeHandlePtr(new ros::NodeHandle);

    std::shared_ptr<fluid::InitState> init_state = std::make_shared<fluid::InitState>(node_handle_p, refresh_rate);
    std::shared_ptr<fluid::IdleState> idle_state = std::make_shared<fluid::IdleState>(node_handle_p, refresh_rate);
    std::shared_ptr<fluid::TakeOffState> take_off_state = std::make_shared<fluid::TakeOffState>(node_handle_p, refresh_rate);
    std::shared_ptr<fluid::LandState> land_state = std::make_shared<fluid::LandState>(node_handle_p, refresh_rate);
    std::shared_ptr<fluid::HoldState> hold_state = std::make_shared<fluid::HoldState>(node_handle_p, refresh_rate);
    std::shared_ptr<fluid::MoveState> move_state = std::make_shared<fluid::MoveState>(node_handle_p, refresh_rate);

    std::vector<Edge> edges;

    current_state_p = init_state;
    edges.emplace_back(Edge(init_state, idle_state));
    edges.emplace_back(Edge(idle_state, take_off_state));
    edges.emplace_back(Edge(take_off_state, hold_state));
    edges.emplace_back(Edge(hold_state, move_state));
    edges.emplace_back(Edge(move_state, hold_state));
    edges.emplace_back(Edge(hold_state, land_state));
    edges.emplace_back(Edge(land_state, idle_state));

    addEdges(edges);

    initialized_ = true;
}

ros::NodeHandlePtr fluid::Graph::getNodeHandlePtr() {
    return node_handle_p;
}

