/**
 * @file operation_handler.h
 */

#ifndef OPERATION_HANDLER_H
#define OPERATION_HANDLER_H

#include <ros/ros.h>
#include <memory>

#include <fluid/Explore.h>
#include <fluid/TakeOff.h>
#include <fluid/Land.h>
#include <fluid/Extract.h>
#include <fluid/Travel.h>

#include "status_publisher.h"

/** 
 * @brief Listens to service requests to the state machine and handles the them. These service requests are defined as
 *        an *operation* in the state machine. E.g. take off is an operation. 
 */
class OperationHandler {
   private:
    const int REFRESH_RATE = 20;

    /**
     * @brief The current state being executed.
     */
    std::shared_ptr<State> current_state_ptr;

    /**
     * @brief Holds the current state of the FSM and publishes it.
     */
    StatusPublisher status_publisher;

    /**
     * @brief Used to initialize the service servers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The servers which are called when requesting a new operation. 
     */
    ros::ServiceServer take_off_server, travel_server, explore_server, extract_server, land_server;

    /**
     * @brief Service handler for the take off service.
     * 
     * @param request The take off request.
     * @param response The take off response. 
     * 
     * @return true When the service call has been handled.
     */
    bool take_off(fluid::TakeOff::Request& request, fluid::TakeOff::Response& response);

    /**
     * @brief Service handler for the travel service.
     * 
     * @param request The travel request.
     * @param response The travel response. 
     * 
     * @return true When the service call has been handled.
     */
    bool travel(fluid::Travel::Request& request, fluid::Travel::Response& response);

    /**
     * @brief Service handler for the explore service.
     * 
     * @param request The explore request.
     * @param response The explore response. 
     * 
     * @return true When the service call has been handled.
     */
    bool explore(fluid::Explore::Request& request, fluid::Explore::Response& response);

    /**
     * @brief Service handler for the extract service.
     * 
     * @param request The extract request.
     * @param response The extract response. 
     * 
     * @return true When the service call has been handled.
     */
    bool extract(fluid::Extract::Request& request, fluid::Extract::Response& response);

    /**
     * @brief Service handler for the land service.
     * 
     * @param request The land request.
     * @param response The land response. 
     * 
     * @return true When the service call has been handled.
     */
    bool land(fluid::Land::Request& request, fluid::Land::Response& response);

    /**
     * @brief Validates if a given operation to @p target_state_identifier is valid from @p current_state_identifier. 
     * 
     * @param current_state_identifier The current state identifier.
     * @param target_state_identifier The target state identifier of the operation.
     * 
     * @return true if the operation is valid.
     */
    bool isValidOperation(StateIdentifier current_state_identifier, StateIdentifier target_state_identifier) const;

    /**
     * @brief Performs the transition to the given @p state.
     * 
     * @param target_state_identifier The state to transition to. 
     */
    void transitionToState(StateIdentifier target_state_identifier);

   public:
};

#endif
