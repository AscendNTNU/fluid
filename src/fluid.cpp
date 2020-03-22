/**
 * @file fluid.cpp
 */

#include "fluid.h"

#include <fluid/OperationCompletion.h>

#include "explore_state.h"
#include "extract_module_state.h"
#include "fluid.h"
#include "follow_mast_state.h"
#include "hold_state.h"
#include "land_state.h"
#include "mavros_interface.h"
#include "take_off_state.h"
#include "travel_state.h"
#include "util.h"

/******************************************************************************************************
 *                                          Singleton                                                 *
 ******************************************************************************************************/

std::shared_ptr<Fluid> Fluid::instance_ptr;

void Fluid::initialize(const FluidConfiguration configuration) {
    if (!instance_ptr) {
        // Can't use std::make_shared here as the constructor is private.
        instance_ptr = std::shared_ptr<Fluid>(new Fluid(configuration));
    }
}

Fluid& Fluid::getInstance() { return *instance_ptr; }

std::shared_ptr<StatusPublisher> Fluid::getStatusPublisherPtr() { return status_publisher_ptr; }

/******************************************************************************************************
 *                                          Operations                                                *
 ******************************************************************************************************/

bool Fluid::take_off(fluid::TakeOff::Request& request, fluid::TakeOff::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        StateIdentifier::TAKE_OFF, {std::make_shared<TakeOffState>(request.height), std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::travel(fluid::Travel::Request& request, fluid::Travel::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        StateIdentifier::TRAVEL, {std::make_shared<TravelState>(request.path), std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::explore(fluid::Explore::Request& request, fluid::Explore::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        StateIdentifier::EXPLORE, {std::make_shared<ExploreState>(request.path), std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::extractModule(fluid::ExtractModule::Request& request, fluid::ExtractModule::Response& response) {
    Response attempt_response = attemptToCreateOperation(
        StateIdentifier::EXTRACT_MODULE, {std::make_shared<ExtractModuleState>(), std::make_shared<HoldState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

bool Fluid::land(fluid::Land::Request& request, fluid::Land::Response& response) {
    Response attempt_response =
        attemptToCreateOperation(StateIdentifier::LAND, {std::make_shared<LandState>(), std::make_shared<LandState>()});

    response.message = attempt_response.message;
    response.success = attempt_response.success;
    return true;
}

Fluid::Response Fluid::attemptToCreateOperation(const StateIdentifier& target_state_identifier,
                                                const std::list<std::shared_ptr<State>>& execution_queue) {
    Response response;
    StateIdentifier current_state_identifier = getStateIdentifierForState(current_state_ptr);

    response.success = isValidOperation(current_state_identifier, target_state_identifier);

    if (!response.success) {
        response.message = "Cannot transition to " + getStringFromStateIdentifier(target_state_identifier) + " from " +
                           getStringFromStateIdentifier(current_state_identifier);
        return response;
    } else {
        ROS_INFO_STREAM(ros::this_node::getName().c_str()
                        << ": "
                        << "Transitioning to " << getStringFromStateIdentifier(target_state_identifier).c_str());
    }

    got_new_operation = true;
    state_execution_queue = execution_queue;
    current_operation = getStringFromStateIdentifier(state_execution_queue.front()->identifier);

    return response;
}

std::shared_ptr<State> Fluid::performStateTransition(std::shared_ptr<State> current_state_ptr,
                                                     std::shared_ptr<State> target_state_ptr) {
    ros::Rate rate(Fluid::getInstance().configuration.refresh_rate);
    MavrosInterface mavros_interface;

    // Loop until the PX4 mode is set.
    const std::string target_state_px4_mode = getPX4ModeForStateIdentifier(target_state_ptr->identifier);
    while (ros::ok() && !mavros_interface.attemptToSetState(target_state_px4_mode)) {
        ros::spinOnce();
        rate.sleep();
    }

    if (current_state_ptr) {
        target_state_ptr->current_pose = current_state_ptr->getCurrentPose();
        target_state_ptr->current_twist = current_state_ptr->getCurrentTwist();
    }

    return target_state_ptr;
}

/******************************************************************************************************
 *                                          Helpers                                                   *
 ******************************************************************************************************/

StateIdentifier Fluid::getStateIdentifierForState(std::shared_ptr<State> state_ptr) {
    if (!state_ptr) {
        return StateIdentifier::UNDEFINED;
    }

    return state_ptr->identifier;
}

bool Fluid::isValidOperation(const StateIdentifier& current_state_identifier,
                             const StateIdentifier& target_state_identifier) const {
    switch (target_state_identifier) {
        case StateIdentifier::TAKE_OFF:
            return current_state_identifier == StateIdentifier::UNDEFINED ||
                   current_state_identifier == StateIdentifier::LAND;

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

/******************************************************************************************************
 *                                          Main Logic                                                *
 ******************************************************************************************************/

void Fluid::run() {
    ros::Rate rate(configuration.refresh_rate);
    bool has_called_completion = false;

    while (ros::ok()) {
        got_new_operation = false;
        ROS_FATAL_STREAM(current_operation);
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
            getStatusPublisherPtr()->status.current_operation = current_operation;
            getStatusPublisherPtr()->status.current_state = getStringFromStateIdentifier(current_state_ptr->identifier);
            getStatusPublisherPtr()->status.px4_mode = getPX4ModeForStateIdentifier(current_state_ptr->identifier);

            current_state_ptr->perform([&]() -> bool { return !got_new_operation; }, state_execution_queue.empty());
        }

        ros::spinOnce();
        rate.sleep();
    }
}
