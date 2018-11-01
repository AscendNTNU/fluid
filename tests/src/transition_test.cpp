//
// Created by simengangstad on 28.10.18.
//

#include <ros/ros.h>

#include "test_state_1.h"
#include "test_state_2.h"
#include "../../include/states/init_state.h"
#include "../../include/states/init_state.h"
#include "../../include/states/init_state.h"
#include "../../include/core/transition.h"

#include <iostream>
#include <memory>

int main(int argc, char** argv) {

    ros::init(argc, argv, "transition_test");

    std::shared_ptr<fluid::InitState> init_state = std::make_shared<fluid::InitState>();
    init_state->perform();

    std::


    /**
    std::shared_ptr<TestState1> test_state_1 = std::make_shared<TestState1>();
    std::shared_ptr<TestState2> test_state_2 = std::make_shared<TestState2>();

    init_state->perform();
    fluid::Transition init_transition(init_state, test_state_1, 20);
    init_transition.perform([] {
       std::cout << "Transitioned" << std::endl;
    });

    test_state_1->perform();
    fluid::Transition transition(test_state_1, test_state_2, 20);
    transition.perform([]() {
        std::cout << "Transitioned" << std::endl;
    });
    test_state_2->perform();
*/
    return 0;
}
