//
// Created by simengangstad on 28.10.18.
//

#include "../include/test_state_1.h"

#include <iostream>
#include <string>

int count = 0;

bool TestState1::hasFinishedExecution() {
    count += 1;
    return count > 10;
}

void TestState1::tick() {
    std::cout << identifier << " ticks" << std::endl;
}