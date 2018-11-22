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

        for(const auto &neighborState : adjacency_list->at(start_state_identifier)) {

            if(!visited[neighborState->identifier]) {
                visited[neighborState->identifier] = true;
                queue.push_back(neighborState->identifier);
            }
        }
    }


    // Transform plan of state identifiers into a plan of states
    std::vector<std::shared_ptr<fluid::State>> states_in_plan;

    for (const auto &identifier : plan) {
        states_in_plan.push_back(getStateWithIdentifier(identifier));
    }

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

void fluid::Graph::initialize() {

    // TODO: new here?
    node_handle_p = ros::NodeHandlePtr(new ros::NodeHandle);

    std::shared_ptr<fluid::InitState> init_state        = std::make_shared<fluid::InitState>(node_handle_p);
    std::shared_ptr<fluid::IdleState> idle_state        = std::make_shared<fluid::IdleState>(node_handle_p);
    std::shared_ptr<fluid::TakeOffState> take_off_state = std::make_shared<fluid::TakeOffState>(node_handle_p);
    std::shared_ptr<fluid::LandState> land_state        = std::make_shared<fluid::LandState>(node_handle_p);
    std::shared_ptr<fluid::HoldState> hold_state        = std::make_shared<fluid::HoldState>(node_handle_p);
    std::shared_ptr<fluid::MoveState> move_state        = std::make_shared<fluid::MoveState>(node_handle_p);

    std::vector<Edge> edges;

    current_state_p = init_state;
    edges.emplace_back(Edge(init_state, idle_state));
    edges.emplace_back(Edge(idle_state, take_off_state));
    edges.emplace_back(Edge(take_off_state, hold_state));
    edges.emplace_back(Edge(hold_state, move_state));
    edges.emplace_back(Edge(move_state, hold_state));
    edges.emplace_back(Edge(move_state, land_state));
    edges.emplace_back(Edge(hold_state, land_state));
    edges.emplace_back(Edge(land_state, idle_state));

    addEdges(edges);

    initialized_ = true;
}

ros::NodeHandlePtr fluid::Graph::getNodeHandlePtr() {
    return node_handle_p;
}

