//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"
#include <ros/ros.h>
#include <list>
#include <fluid_fsm/OperationGoal.h>
#include <fluid_fsm/OperationAction.h>
#include "actionlib/operation_server.h"

#include "../include/states/init_state.h"
#include "../include/states/take_off_state.h"
#include "../include/states/hold_state.h"
#include "../include/core/state.h"
#include "../include/core/transition.h"

#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");

    // Initial states

    ros::NodeHandlePtr node_handle_p(new ros::NodeHandle);

    /*
    std::shared_ptr<fluid::InitState> init_state = std::make_shared<fluid::InitState>(node_handle_p);
    init_state->perform([]() -> bool {
        return false;
    });

    std::shared_ptr<fluid::TakeOffState> take_off_state = std::make_shared<fluid::TakeOffState>(node_handle_p);
    fluid::Transition take_off_transition(node_handle_p, init_state, take_off_state, 20);
    take_off_transition.perform([] {
        ROS_INFO("Transitioned to take off");
    });
    take_off_state->position_target.position.z = 2.0;
    take_off_state->perform([]() -> bool {
        return false;
    });
*/


/*
    // Operation

    std::shared_ptr<fluid::State> last_state;
    std::shared_ptr<fluid::Operation> current_operation;
    std::shared_ptr<fluid::Operation> next_operation;

    bool newOperationRequested = false;

    fluid::OperationServer<fluid_fsm::InitAction, fluid_fsm::InitGoalConstPtr> init_server(fluid::OperationIdentifier::init);
    init_server.operationRequestedCallback = [&](std::shared_ptr<fluid::Operation> operation) {
        next_operation = operation;
        newOperationRequested = true;
    };

    fluid::OperationServer<fluid_fsm::TakeOffAction, fluid_fsm::TakeOffGoalConstPtr> take_off_server(fluid::OperationIdentifier::take_off);
    take_off_server.operationRequestedCallback = [&](std::shared_ptr<fluid::Operation> operation) {
        next_operation = operation;
        newOperationRequested = true;
    };

    fluid::OperationServer<fluid_fsm::MoveAction, fluid_fsm::MoveGoalConstPtr> move_server(fluid::OperationIdentifier::move);
    move_server.operationRequestedCallback = [&](std::shared_ptr<fluid::Operation> operation) {
        next_operation = operation;
        newOperationRequested = true;
    };

    fluid::OperationServer<fluid_fsm::LandAction, fluid_fsm::LandGoalConstPtr> land_server(fluid::OperationIdentifier::land);
    land_server.operationRequestedCallback = [&](std::shared_ptr<fluid::Operation> operation) {
        next_operation = operation;
        newOperationRequested = true;
    };




    /*

    // Hold state

    last_state = std::make_shared<fluid::HoldState>(node_handle_p);
    fluid::Transition hold_transition(node_handle_p, take_off_state, last_state, 20);
    hold_transition.perform([] {
        ROS_INFO("Transitioned to hold");
    });

    last_state->position_target.position.z = 2.0;

*/

/*

    // Operation and last state logic

    ROS_INFO_STREAM("Fluid launched, waiting for requests...");

    ros::Rate rate(20);

    while (ros::ok()) {

        if (newOperationRequested) {
            current_operation = next_operation;
            next_operation.reset();
            newOperationRequested = false;
        }

        // We have an operation to execute.
        if (current_operation) {
            current_operation->perform(
                    [&]() -> bool {
                        // We abort current mission if there is a new operation.
                        if (newOperationRequested) {
                            ROS_INFO_STREAM("Aborting current operation" << current_operation->identifier.c_str());
                        }

                        return newOperationRequested;
                    },

                    [&](bool completed) {
                        // Tell respective operation server that we finished executing
                        if (completed) {
                            if (current_operation->identifier == "init_operation") {
                                init_server.finishedExecuting();
                                ROS_INFO_STREAM("Init operation finished executing");
                            }
                            if (current_operation->identifier == "take_off_operation") {
                                take_off_server.finishedExecuting();
                                ROS_INFO_STREAM("Take off operation finished executing");
                            }
                            if (current_operation->identifier == "move_operation") {
                                move_server.finishedExecuting();
                                ROS_INFO_STREAM("Move operation finished executing");
                            }
                            if (current_operation->identifier == "land_operation") {
                                land_server.finishedExecuting();
                                ROS_INFO_STREAM("Land operation finished executing");
                            }
                        }
                    });

            last_state = current_operation->getFinalStatePtr();
            current_operation.reset();
            ROS_INFO_STREAM("Operation finished, state is now: " << last_state->identifier);
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {
            if (last_state) {
                last_state->perform([&]() -> bool {
                    // We abort the execution of the current state if there is a new operation.
                    if (newOperationRequested) {
                        ROS_INFO_STREAM(
                                "Aborting state for new operation operation: " << next_operation->identifier.c_str());
                    }

                    return newOperationRequested;
                });
            }
        }

        ros::spinOnce();
        rate.sleep();
    }*/


    return 0;
}
