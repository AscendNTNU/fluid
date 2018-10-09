//
// Created by simengangstad on 27.09.18.
//

#include "../include/state.h"
#include "../include/transition.h"
#include "../include/operation.h"
#include "../include/graph.h"
#include <iostream>

class IdleState: public State {
public:

    IdleState(Pose pose) : State("idle", pose) {}

    void perform() {
        if (auto state_delegate = state_delegate_p.lock()) {
            state_delegate->stateBegan(*this);
        }

        std::cout << "Idle..." << std::endl;

        if (auto state_delegate = state_delegate_p.lock()) {
            state_delegate->stateFinished(*this);
        }
    }
};

class MoveState: public State {
public:

    MoveState(Pose pose) : State("move", pose) {}

    void perform() {
        if (auto state_delegate = state_delegate_p.lock()) {
            state_delegate->stateBegan(*this);
        }

        std::cout << "Moving..." << std::endl;

        if (auto state_delegate = state_delegate_p.lock()) {
            state_delegate->stateFinished(*this);
        }
    }
};


int main(int argc, const char** argv) {

    /*
    std::shared_ptr<Operation> operation;
    
    operation = std::make_shared<Operation>();
    
    operation->addState(std::make_shared<MoveState>(pose));
    operation->addState(std::make_shared<IdleState>(pose));
    
    operation->perform();
*/
    Pose pose;
    
    std::shared_ptr<MoveState> move_state = std::make_shared<MoveState>(pose);
    std::shared_ptr<IdleState> idle_state = std::make_shared<IdleState>(pose);
    std::vector<Edge> edges;
    
    edges.push_back(Edge(idle_state, move_state));
    edges.push_back(Edge(move_state, idle_state));
    
    Graph graph(edges);
    
    graph.print();
    
    return 0;
}
