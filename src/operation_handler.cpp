/**
 * @file operation_handler.cpp
 */

#include "operation_handler.h"

#include <assert.h>
#include <fluid/OperationCompletion.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>

#include "core.h"
#include "explore_state.h"
#include "extract_module_state.h"
#include "follow_mast_state.h"
#include "hold_state.h"
#include "land_state.h"
#include "mavros_interface.h"
#include "take_off_state.h"
#include "travel_state.h"
#include "util.h"

OperationHandler::OperationHandler() {
    take_off_server = node_handle.advertiseService("fluid/take_off", &OperationHandler::take_off, this);
    travel_server = node_handle.advertiseService("fluid/travel", &OperationHandler::travel, this);
    explore_server = node_handle.advertiseService("fluid/explore", &OperationHandler::explore, this);
    extract_module_server = node_handle.advertiseService("fluid/extract_module", &OperationHandler::extractModule, this);
    land_server = node_handle.advertiseService("fluid/land", &OperationHandler::land, this);
    operation_completion_client = node_handle.serviceClient<fluid::OperationCompletion>("fluid/operation_completion");
}

bool OperationHandler::take_off(fluid::TakeOff::Request& request, fluid::TakeOff::Response& response) {
    Response attempt_response = attemptToCreateOperation(StateIdentifier::TAKE_OFF,
                                                         {std::make_shared<TakeOffState>(request.height),
                                                          std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool OperationHandler::travel(fluid::Travel::Request& request, fluid::Travel::Response& response) {
    Response attempt_response = attemptToCreateOperation(StateIdentifier::TRAVEL,
                                                         {std::make_shared<TravelState>(request.path),
                                                          std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool OperationHandler::explore(fluid::Explore::Request& request, fluid::Explore::Response& response) {
    Response attempt_response = attemptToCreateOperation(StateIdentifier::EXPLORE,
                                                         {std::make_shared<ExploreState>(request.path),
                                                          std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool OperationHandler::extractModule(fluid::ExtractModule::Request& request, fluid::ExtractModule::Response& response) {
    Response attempt_response = attemptToCreateOperation(StateIdentifier::EXTRACT_MODULE,
                                                         {std::make_shared<ExtractModuleState>(),
                                                          std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool OperationHandler::land(fluid::Land::Request& request, fluid::Land::Response& response) {
    Response attempt_response = attemptToCreateOperation(StateIdentifier::LAND,
                                                         {std::make_shared<LandState>(),
                                                          std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

OperationHandler::Response OperationHandler::attemptToCreateOperation(const StateIdentifier& target_state_identifier,
                                                                      const std::list<std::shared_ptr<State>>& state_execution_queue) {
    Response response;
    StateIdentifier current_state_identifier = getStateIdentifierForState(current_state_ptr);

    response.success = isValidOperation(current_state_identifier, target_state_identifier);

    if (!response.success) {
        response.message = "Cannot transition to " +
                           getStringFromStateIdentifier(target_state_identifier) +
                           " from " +
                           getStringFromStateIdentifier(current_state_identifier);
        return response;
    }

    got_new_operation = true;
    this->state_execution_queue = state_execution_queue;
    current_operation = getStringFromStateIdentifier(this->state_execution_queue.front()->identifier);

    return response;
}

StateIdentifier OperationHandler::getStateIdentifierForState(std::shared_ptr<State> state_ptr) {
    if (!state_ptr) {
        return StateIdentifier::UNDEFINED;
    }

    return state_ptr->identifier;
}

bool OperationHandler::isValidOperation(const StateIdentifier& current_state_identifier,
                                        const StateIdentifier& target_state_identifier) const {
    switch (target_state_identifier) {
        case StateIdentifier::TAKE_OFF:
            return current_state_identifier == StateIdentifier::UNDEFINED;

        case StateIdentifier::EXPLORE:
        case StateIdentifier::TRAVEL:
            return current_state_identifier != StateIdentifier::TAKE_OFF &&
                   current_state_identifier != StateIdentifier::LAND &&
                   current_state_identifier != StateIdentifier::UNDEFINED;

        case StateIdentifier::LAND:
            return current_state_identifier != StateIdentifier::TAKE_OFF &&
                   current_state_identifier != StateIdentifier::UNDEFINED;

        case StateIdentifier::EXTRACT_MODULE:
            return current_state_identifier != StateIdentifier::TAKE_OFF &&
                   current_state_identifier != StateIdentifier::LAND &&
                   current_state_identifier != StateIdentifier::UNDEFINED;
        default:
            return false;
    }
}

std::shared_ptr<State> OperationHandler::performStateTransition(std::shared_ptr<State> current_state_ptr,
                                                                std::shared_ptr<State> target_state_ptr) {
    ros::Rate rate(REFRESH_RATE);
    MavrosInterface mavros_interface;

    // Loop until the PX4 mode is set.
    while (ros::ok() && !mavros_interface.attemptToSetState(getPX4ModeForStateIdentifier(target_state_ptr->identifier))) {
        ros::spinOnce();
        rate.sleep();
    }

    if (current_state_ptr) {
        target_state_ptr->current_pose = current_state_ptr->getCurrentPose();
    }

    return target_state_ptr;
}

void OperationHandler::run() {
    ros::Rate rate(REFRESH_RATE);
    bool has_called_completion = false;

    while (ros::ok()) {
        got_new_operation = false;

        if (!state_execution_queue.empty()) {
            current_state_ptr = performStateTransition(current_state_ptr, state_execution_queue.front());
            state_execution_queue.pop_front();
            has_called_completion = false;
        }

        // If we are at the steady state, we call the completion service
        if (state_execution_queue.empty() && !has_called_completion) {
            fluid::OperationCompletion operation_completion;
            operation_completion.request.operation = current_operation;
            operation_completion_client.call(operation_completion);
            has_called_completion = true;
        }

        if (current_state_ptr) {
            Core::getStatusPublisherPtr()->status.current_operation = current_operation;
            Core::getStatusPublisherPtr()->status.current_state = getStringFromStateIdentifier(current_state_ptr->identifier);
            Core::getStatusPublisherPtr()->status.px4_mode = getPX4ModeForStateIdentifier(current_state_ptr->identifier);

            current_state_ptr->perform([&]() -> bool { return !got_new_operation; }, state_execution_queue.empty());
        }

        ros::spinOnce();
        rate.sleep();
    }
}