//
// Created by simengangstad on 11.10.18.
//

#include "../../../include/core/operation/state_graph.h"

#include "../../../include/states/init_state.h"
#include "../../../include/states/idle_state.h"
#include "../../../include/states/take_off_state.h"
#include "../../../include/states/land_state.h"
#include "../../../include/states/hold_state.h"
#include "../../../include/states/move_state.h"
#include <vector>
#include <states/init_state.h>

bool fluid::StateGraph::isInitialized() {
    return initialized_;
}

void fluid::StateGraph::initialize() {

    node_handle_p = ros::NodeHandlePtr(new ros::NodeHandle);

    std::shared_ptr<fluid::InitState> init_state        = std::make_shared<fluid::InitState>(node_handle_p);
    std::shared_ptr<fluid::IdleState> idle_state        = std::make_shared<fluid::IdleState>(node_handle_p);
    std::shared_ptr<fluid::TakeOffState> take_off_state = std::make_shared<fluid::TakeOffState>(node_handle_p);
    std::shared_ptr<fluid::LandState> land_state        = std::make_shared<fluid::LandState>(node_handle_p);
    std::shared_ptr<fluid::HoldState> hold_state        = std::make_shared<fluid::HoldState>(node_handle_p);
    std::shared_ptr<fluid::MoveState> move_state        = std::make_shared<fluid::MoveState>(node_handle_p);

    std::vector<Edge> edges;

    current_state_p = idle_state;
    edges.emplace_back(idle_state, take_off_state);
    edges.emplace_back(take_off_state, hold_state);
    edges.emplace_back(Edge(hold_state, move_state));
    edges.emplace_back(Edge(move_state, hold_state));
    edges.emplace_back(Edge(hold_state, land_state));
    edges.emplace_back(Edge(land_state, idle_state));

    addEdges(edges);

    initialized_ = true;
}

ros::NodeHandlePtr fluid::StateGraph::getNodeHandlePtr() {
    return node_handle_p;
}
