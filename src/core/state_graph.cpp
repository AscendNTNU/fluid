//
//  Created by Simen Gangstad on 24/01/2019.
//

#include "state_graph.h"

#include <iterator>
#include <algorithm>
#include <climits>

#include "init_state.h"
#include "idle_state.h"
#include "take_off_state.h"
#include "land_state.h"
#include "hold_state.h"
#include "move_state.h"
#include "position_follow_state.h"

fluid::StateGraph::StateGraph() {

    adjacency_list_ptr_ = std::make_unique<AdjacencyList>();

    // Set up the graph
    std::shared_ptr<fluid::State> init_state = std::make_shared<fluid::InitState>();
    std::shared_ptr<fluid::State> idle_state = std::make_shared<fluid::IdleState>();
    std::shared_ptr<fluid::State> take_off_state = std::make_shared<fluid::TakeOffState>();
    std::shared_ptr<fluid::State> land_state = std::make_shared<fluid::LandState>();
    std::shared_ptr<fluid::State> hold_state = std::make_shared<fluid::HoldState>();
    std::shared_ptr<fluid::State> move_state = std::make_shared<fluid::MoveState>();
    std::shared_ptr<fluid::State> position_follow_state = std::make_shared<fluid::PositionFollowState>();

    std::vector<fluid::Edge<std::shared_ptr<fluid::State>>> edges;

    current_state_ptr = std::dynamic_pointer_cast<fluid::State>(init_state);
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(init_state, idle_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(idle_state, take_off_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(take_off_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(hold_state, move_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(move_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(hold_state, land_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(land_state, idle_state));

    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(hold_state, position_follow_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(position_follow_state, hold_state));
    addEdges(edges);
}

void fluid::StateGraph::addEdges(std::vector<fluid::Edge<std::shared_ptr<State>>> edges) {
    for (auto edge : edges) {

        // If the node isn't added in the adjacency list, we add the the node and its neighbour (edge destination).
        if (adjacency_list_ptr_->find(edge.source->identifier) == adjacency_list_ptr_->end()) {

            std::vector<std::shared_ptr<fluid::State>> neighbours;
            neighbours.push_back(edge.destination);
            adjacency_list_ptr_->insert(std::make_pair(edge.source->identifier, neighbours));

            // As this node didn't exist in the graph, we add it the list of nodes
            states_.push_back(edge.source);
        }
        // Node exists in graph, just add the new neighbour
        else {
            adjacency_list_ptr_->at(edge.source->identifier).push_back(edge.destination);
        }
    }
}

std::vector<std::shared_ptr<fluid::State>> fluid::StateGraph::getStates() {
    return states_;
}

std::vector<std::shared_ptr<fluid::State>> fluid::StateGraph::getPathToEndState(std::string start_state_identifier,
                                                                                std::string end_state_identifier) {
    
    // BFS
    std::list<std::string> queue;
    std::map<std::string, bool> visited;
    std::map<std::string, std::string> pred;

    for (auto const& item : *adjacency_list_ptr_) {
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

        for(const auto &neighborState : adjacency_list_ptr_->at(start_state_identifier)) {
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


    // Get shortest back by backtracing

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

std::shared_ptr<fluid::State> fluid::StateGraph::getStateWithIdentifier(std::string identifier) {
    // Get state with this identifier from the state vector
    for (auto state : getStates()) {
        if (state->identifier == identifier) {
            return state;
        }
    }

    return nullptr;
}

std::ostream& fluid::StateGraph::operator<<(std::ostream& ostream) {
for (auto const& item : *adjacency_list_ptr_) {
        
        ostream << "\n Adjacency list of vertex " << item.first << "\n head ";
        
        for (auto &neighbor : item.second) {
            ostream << "-> " << neighbor->identifier;
        }
    }

    return ostream;
}

