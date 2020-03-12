#include "state_graph.h"

#include <algorithm>
#include <climits>
#include <iterator>

#include "explore_state.h"
#include "extract_module_state.h"
#include "follow_mast_state.h"
#include "hold_state.h"
#include "idle_state.h"
#include "init_state.h"
#include "land_state.h"
#include "take_off_state.h"
#include "travel_state.h"

StateGraph::StateGraph() {
    adjacency_list_ptr = std::make_unique<AdjacencyList>();

    // Set up the graph
    std::shared_ptr<State> init_state = std::make_shared<InitState>();
    std::shared_ptr<State> idle_state = std::make_shared<IdleState>();
    std::shared_ptr<State> take_off_state = std::make_shared<TakeOffState>();
    std::shared_ptr<State> land_state = std::make_shared<LandState>();
    std::shared_ptr<State> hold_state = std::make_shared<HoldState>();
    std::shared_ptr<State> explore_state = std::make_shared<ExploreState>();
    std::shared_ptr<State> travel_state = std::make_shared<TravelState>();
    std::shared_ptr<State> extract_module_state = std::make_shared<ExtractModuleState>();
    std::shared_ptr<State> follow_mast_state = std::make_shared<FollowMastState>();

    std::vector<Edge<std::shared_ptr<State>>> edges;

    current_state_ptr = init_state;
    edges.emplace_back(Edge<std::shared_ptr<State>>(init_state, idle_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(idle_state, take_off_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(take_off_state, hold_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(hold_state, explore_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(explore_state, hold_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(hold_state, travel_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(travel_state, hold_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(hold_state, land_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(land_state, idle_state));

    edges.emplace_back(Edge<std::shared_ptr<State>>(hold_state, follow_mast_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(follow_mast_state, hold_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(follow_mast_state, extract_module_state));
    edges.emplace_back(Edge<std::shared_ptr<State>>(extract_module_state, hold_state));

    addEdges(edges);
}

void StateGraph::addEdges(const std::vector<Edge<std::shared_ptr<State>>>& edges) {
    for (auto edge : edges) {
        // If the node isn't added in the adjacency list, we add the the node and its neighbour (edge destination).
        if (adjacency_list_ptr->find(edge.source->identifier) == adjacency_list_ptr->end()) {
            std::vector<std::shared_ptr<State>> neighbours;
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

std::vector<std::shared_ptr<State>> StateGraph::getStates() const { return states; }

std::vector<std::shared_ptr<State>> StateGraph::getPathToEndState(const StateIdentifier& start_state_identifier,
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
        return std::vector<std::shared_ptr<State>>();
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

        for (const auto& neighborState : adjacency_list_ptr->at(start)) {
            if (!visited[neighborState->identifier]) {
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
    std::vector<std::shared_ptr<State>> states_in_plan;

    bool includes_move = false;

    for (const auto& identifier : path) {
        states_in_plan.push_back(getStateWithIdentifier(identifier));

        if (identifier == StateIdentifier::Explore || identifier == StateIdentifier::Travel) {
            includes_move = true;
        }
    }

    std::reverse(states_in_plan.begin(), states_in_plan.end());

    // If we should include a move and the path include a state after hold, we have to include a move state.
    // This is of course just in the case the path doesn't already include a move.
    if (!includes_move && should_include_move) {
        auto iterator = std::find_if(
            states_in_plan.begin(), states_in_plan.end(),
            [](const std::shared_ptr<State>& state) { return state->identifier == StateIdentifier::Hold; });

        if (iterator != states_in_plan.end()) {
            states_in_plan.insert(iterator + 1, getStateWithIdentifier(StateIdentifier::Explore));
        }
    }

    return states_in_plan;
}

bool StateGraph::areConnected(const StateIdentifier& start_state_identifier,
                              const StateIdentifier& end_state_identifier) const {
    std::vector<std::shared_ptr<State>> path = getPathToEndState(start_state_identifier, end_state_identifier, false);

    if (path.empty()) {
        return false;
    } else {
        return (*path.begin())->identifier == start_state_identifier &&
               (*(path.end() - 1))->identifier == end_state_identifier;
    }
}

std::shared_ptr<State> StateGraph::getStateWithIdentifier(const StateIdentifier& identifier) const {
    for (auto state : getStates()) {
        if (state->identifier == identifier) {
            return state;
        }
    }

    return nullptr;
}