#include "state_graph.h"

#include <iterator>
#include <algorithm>
#include <climits>

#include "init_state.h"
#include "idle_state.h"
#include "take_off_state.h"
#include "land_state.h"
#include "hold_state.h"
#include "explore_state.h"
#include "travel_state.h"
#include "rotate_state.h"

fluid::StateGraph::StateGraph() {

    adjacency_list_ptr = std::make_unique<AdjacencyList>();

    // Set up the graph
    std::shared_ptr<fluid::State> init_state = std::make_shared<fluid::InitState>();
    std::shared_ptr<fluid::State> idle_state = std::make_shared<fluid::IdleState>();
    std::shared_ptr<fluid::State> take_off_state = std::make_shared<fluid::TakeOffState>();
    std::shared_ptr<fluid::State> land_state = std::make_shared<fluid::LandState>();
    std::shared_ptr<fluid::State> hold_state = std::make_shared<fluid::HoldState>();
    std::shared_ptr<fluid::State> explore_state = std::make_shared<fluid::ExploreState>();
    std::shared_ptr<fluid::State> travel_state = std::make_shared<fluid::TravelState>();
    std::shared_ptr<fluid::State> rotate_state = std::make_shared<fluid::RotateState>();

    std::vector<fluid::Edge<std::shared_ptr<fluid::State>>> edges;

    current_state_ptr = init_state;
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(init_state, idle_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(idle_state, take_off_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(take_off_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(hold_state, rotate_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(rotate_state, explore_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(explore_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(rotate_state, travel_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(travel_state, hold_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(hold_state, land_state));
    edges.emplace_back(fluid::Edge<std::shared_ptr<fluid::State>>(land_state, idle_state));

    addEdges(edges);
}

void fluid::StateGraph::addEdges(const std::vector<fluid::Edge<std::shared_ptr<State>>>& edges) {
    for (auto edge : edges) {

        // If the node isn't added in the adjacency list, we add the the node and its neighbour (edge destination).
        if (adjacency_list_ptr->find(edge.source->identifier) == adjacency_list_ptr->end()) {

            std::vector<std::shared_ptr<fluid::State>> neighbours;
            neighbours.push_back(edge.destination);
            adjacency_list_ptr->insert(std::make_pair(edge.source->identifier, neighbours));

            // As this node didn't exist in the graph, we add it the list of nodes
            states.push_back(edge.source);
        }
        // Node exists in graph, just add the new neighbour
        else {
            adjacency_list_ptr->at(edge.source->identifier).push_back(edge.destination);
        }
    }
}

std::vector<std::shared_ptr<fluid::State>> fluid::StateGraph::getStates() const {
    return states;
}

std::vector<std::shared_ptr<fluid::State>> fluid::StateGraph::getPathToEndState(const StateIdentifier& start_state_identifier,
                                                                                const StateIdentifier& end_state_identifier,
                                                                                const bool& should_include_move) const {

    bool state_in_graph = false;

    for (auto state : getStates()) {
        if (state->identifier == end_state_identifier) {
            state_in_graph = true;

            break;
        }
    }

    if (!state_in_graph) {
        ROS_FATAL_STREAM("Destination state (" << StateIdentifierStringMap.at(end_state_identifier) 
                                               << ") is not in the graph. Did you spell it correctly?");
        return std::vector<std::shared_ptr<fluid::State>>();
    }

    // BFS
    std::list<StateIdentifier> queue;
    std::map<StateIdentifier, bool> visited;
    std::map<StateIdentifier, StateIdentifier> pred;

    for (auto const& item : *adjacency_list_ptr) {
        visited[item.first] = false;
        pred[item.first] = StateIdentifier::Null; 
    }

    StateIdentifier start = start_state_identifier;

    visited[start] = true;
    queue.push_back(start);
    
    std::vector<std::string> plan;

    bool searching = true;

    while (!queue.empty() && searching) {

        start = queue.front();
        queue.pop_front();

        for(const auto &neighborState : adjacency_list_ptr->at(start)) {
            if(!visited[neighborState->identifier]) {
                visited[neighborState->identifier] = true;
                pred[neighborState->identifier] = start;

                queue.push_back(neighborState->identifier);

                if (neighborState->identifier == end_state_identifier) {
                    searching = false;

                    break;
                }
            }
        }
    }

    // Get shortest back by backtracing

    std::vector<StateIdentifier> path;
    StateIdentifier crawl = end_state_identifier;

    path.push_back(end_state_identifier);

    while (pred[crawl] != StateIdentifier::Null) {
        path.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    // Transform plan of state identifiers into a plan of states
    std::vector<std::shared_ptr<fluid::State>> states_in_plan;

    bool includes_move = false;

    for (const auto &identifier : path) {
        states_in_plan.push_back(getStateWithIdentifier(identifier));

        if (identifier == fluid::StateIdentifier::Explore || identifier == fluid::StateIdentifier::Travel) {
            includes_move = true;
        }
    }
   
    std::reverse(states_in_plan.begin(), states_in_plan.end());

    // If we should include a move and the path include a state after hold, we have to include a move state.
    // This is ofcourse just in the case the path don't already include a move.
    if (!includes_move && should_include_move) {
        auto iterator = std::find_if(states_in_plan.begin(), states_in_plan.end(), [](const std::shared_ptr<fluid::State>& state) {
            return state->identifier == fluid::StateIdentifier::Hold;
        });

        if (iterator != states_in_plan.end()) {
            iterator = states_in_plan.insert(iterator + 1, getStateWithIdentifier(fluid::StateIdentifier::Rotate));
            states_in_plan.insert(iterator + 1, getStateWithIdentifier(fluid::StateIdentifier::Explore));
        }
    }


    return states_in_plan;
}

bool fluid::StateGraph::areConnected(const StateIdentifier& start_state_identifier, const StateIdentifier& end_state_identifier) const {
    std::vector<std::shared_ptr<fluid::State>> path = getPathToEndState(start_state_identifier, end_state_identifier, false);

    if (path.empty()) {
        return false;
    }
    else {
        return (*path.begin())->identifier == start_state_identifier && (*(path.end() - 1))->identifier == end_state_identifier;
    }
}

std::shared_ptr<fluid::State> fluid::StateGraph::getStateWithIdentifier(const StateIdentifier& identifier) const {
    // Get state with this identifier from the state vector
    for (auto state : getStates()) {
        if (state->identifier == identifier) {
            return state;
        }
    }

    return nullptr;
}