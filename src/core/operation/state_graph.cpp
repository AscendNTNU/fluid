//
//  Created by Simen Gangstad on 24/01/2019.
//

#include "../../../include/core/operation/state_graph.h"
#include "../../../include/core/operation/edge.h"

#include "../../../include/states/init_state.h"
#include "../../../include/states/idle_state.h"
#include "../../../include/states/take_off_state.h"
#include "../../../include/states/land_state.h"
#include "../../../include/states/hold_state.h"
#include "../../../include/states/move_state.h"

#include <iterator>
#include <algorithm>
#include <climits>
#include <iostream>


StateGraph::StateGraph() : Graph() {

    std::shared_ptr<fluid::Identifiable> init_state = std::make_shared<fluid::InitState>(refresh_rate);
    std::shared_ptr<fluid::Identifiable> idle_state = std::make_shared<fluid::IdleState>(refresh_rate);
    std::shared_ptr<fluid::Identifiable> take_off_state = std::make_shared<fluid::TakeOffState>(refresh_rate);
    std::shared_ptr<fluid::Identifiable> land_state = std::make_shared<fluid::LandState>(refresh_rate);
    std::shared_ptr<fluid::Identifiable> hold_state = std::make_shared<fluid::HoldState>(refresh_rate);
    std::shared_ptr<fluid::Identifiable> move_state = std::make_shared<fluid::MoveState>(refresh_rate);

    std::vector<fluid::Edge<std::shared_ptr<fluid::Identifiable>>> edges;

    current_state_p = std::dynamic_pointer_cast<fluid::State>(init_state);
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(init_state, idle_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(idle_state, take_off_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(take_off_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(hold_state, move_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(move_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(hold_state, land_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::Identifiable>>(land_state, idle_state));

    addEdges(edges);
}

std::vector<std::shared_ptr<fluid::State>> fluid::StateGraph::getPathToEndState(std::string start_state_identifier,
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
    for (auto node : getNodes()) {
        if (node->identifier == identifier) {

            // TODO: Won't cast for some reason.
            return std::dynamic_pointer_cast<fluid::State>(node);
        }
    }

    return nullptr;
}
