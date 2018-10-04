//
// Created by simengangstad on 27.09.18.
//

#include "../include/state.h"
#include "../include/transition.h"
#include "../include/operation.h"
#include <iostream>

class IdleState: public State {
public:

    IdleState(Pose pose) : State(pose) {}

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

    MoveState(Pose pose) : State(pose) {}

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

    auto operation = std::make_shared<Operation>();
    Pose pose;

    operation->addState(std::make_shared<MoveState>(pose));
    operation->addState(std::make_shared<IdleState>(pose));

    operation->perform();

    return 0;
}