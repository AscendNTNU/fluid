//
// Created by simengangstad on 11.10.18.
//

#include "../../../include/core/operation/state_graph.h"
#include <vector>

fluid::StateGraph::StateGraph(): Graph() {

    /*
    std::shared_ptr<IdleState> idle_state = std::make_shared<IdleState>();
    std::shared_ptr<TakeOffState> take_off_state= std::make_shared<TakeOffState>();
    std::shared_ptr<LandState> land_state = std::make_shared<LandState>();
    std::shared_ptr<HoldState> hold_state = std::make_shared<HoldState>();
    std::shared_ptr<MoveState> move_state = std::make_shared<MoveState>();

    std::vector<Edge> edges;

    current_state_p = idle_state;
    edges.push_back(Edge(idle_state, take_off_state));
    edges.push_back(Edge(take_off_state, hold_state));
    edges.push_back(Edge(hold_state, move_state));
    edges.push_back(Edge(move_state, hold_state));
    edges.push_back(Edge(hold_state, land_state));
    edges.push_back(Edge(land_state, idle_state));

    addEdges(edges);*/
}
