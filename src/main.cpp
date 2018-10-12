//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"

int main(int argc, const char** argv) {

    /*
    std::shared_ptr<Operation> operation;
    
    operation = std::make_shared<Operation>();
    
    operation->addState(std::make_shared<MoveState>(pose));
    operation->addState(std::make_shared<IdleState>(pose));
    
    operation->perform();
*/
    fluid::Operation operation;

    operation.perform();

    return 0;
}
