//
// Created by simengangstad on 28.10.18.
//

#include <ros/ros.h>

#include "test_state_1.h"
#include "test_state_2.h"
#include "../../include/states/init_state.h"
#include "../../include/states/take_off_state.h"
#include "../../include/states/move_state.h"
#include "../../include/core/state.h"
#include "../../include/core/transition.h"

#include <iostream>
#include <memory>
#include <states/land_state.h>
#include <states/idle_state.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "transition_test");

    ros::NodeHandlePtr node_handle_p(new ros::NodeHandle);

    std::shared_ptr<fluid::InitState> init_state = std::make_shared<fluid::InitState>(node_handle_p);
    init_state->perform();

    std::shared_ptr<fluid::TakeOffState> take_off_state = std::make_shared<fluid::TakeOffState>(node_handle_p);
    fluid::Transition init_transition(node_handle_p, init_state, take_off_state, 20);
    init_transition.perform([] {
        ROS_INFO("Transitioned to take off");
    });
    take_off_state->position_target.position.z = 2.0;
    take_off_state->perform();

    std::shared_ptr<fluid::MoveState> move_state = std::make_shared<fluid::MoveState>(node_handle_p);
    move_state->position_target.position.x = 4.0;
    move_state->position_target.position.z = 2.0;
    fluid::Transition move_transition(node_handle_p, take_off_state, move_state, 20);
    move_transition.perform([] {
        ROS_INFO("Transitioned to move");
    });
    move_state->perform();
    ROS_INFO("Finished move");

    std::shared_ptr<fluid::LandState> land_state = std::make_shared<fluid::LandState>(node_handle_p);
    land_state->position_target.position.x = 4.0;
    fluid::Transition land_transition(node_handle_p, move_state, land_state, 20);
    land_transition.perform([] {
        ROS_INFO("Transitioned to land");
    });
    land_state->perform();

    std::shared_ptr<fluid::IdleState> idle_state = std::make_shared<fluid::IdleState>(node_handle_p);
    idle_state->position_target.position = land_state->position_target.position;
    fluid::Transition idle_transition(node_handle_p, land_state, idle_state, 20);
    idle_transition.perform([] {
        ROS_INFO("Transitioned to idle");
    });
    idle_state->perform();


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
