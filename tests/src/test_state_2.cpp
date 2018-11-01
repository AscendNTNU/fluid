//
// Created by simengangstad on 28.10.18.
//

#include "../include/test_state_2.h"

#include <iostream>
#include <string>

int count2 = 0;

bool TestState2::hasFinishedExecution() {
    count2 += 1;
    return count2 > 10;
}

void TestState2::tick() {
    std::cout << identifier << " ticks" << std::endl;
}