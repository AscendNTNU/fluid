//
// Created by simengangstad on 28.10.18.
//

#ifndef FLUID_FSM_TEST_STATE_2_H
#define FLUID_FSM_TEST_STATE_2_H

#include "../../include/core/state.h"
#include "test_pose_publisher.h"
#include <memory>

class TestState2: public fluid::State {

public:
    TestState2() : fluid::State("test_state_2", std::make_shared<TestPosePublisher>(), 20) {}

    bool hasFinishedExecution();

    void tick();
};

#endif //FLUID_FSM_TEST_STATE_2_H
