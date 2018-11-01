//
// Created by simengangstad on 28.10.18.
//

#ifndef FLUID_FSM_TEST_STATE_1_H
#define FLUID_FSM_TEST_STATE_1_H

#include "../../include/core/state.h"
#include "test_pose_publisher.h"
#include <memory>

class TestState1: public fluid::State {

public:
    TestState1() : fluid::State("test_state_1", std::make_shared<TestPosePublisher>(), 20) {}

    bool hasFinishedExecution();

    void tick();
};

#endif //FLUID_FSM_TEST_STATE_1_H
