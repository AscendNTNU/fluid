/**
 * @file operation_handler.h
 */

#ifndef OPERATION_HANDLER_H
#define OPERATION_HANDLER_H

#include <ros/ros.h>
#include <list>
#include <memory>

#include <fluid/Explore.h>
#include <fluid/ExtractModule.h>
#include <fluid/Land.h>
#include <fluid/TakeOff.h>
#include <fluid/Travel.h>

#include "state.h"
#include "status_publisher.h"

/** 
 * @brief Listens to service requests to the state machine and handles the them. These service requests are defined as
 *        an *operation* in the state machine. E.g. take off is an operation. 
 */
class OperationHandler {
   private:
    const int REFRESH_RATE = 20;

    /**
     * @brief Mapping for responses in service calls made by the operaiton handler.
     */
    struct Response {
        /**
         * @brief Whether the service call was successful.
         */
        bool success;

        /**
         * @brief Error messsage (if any)
         */
        std::string message;
    };

    /**
     * @brief The current state being executed
     */
    std::shared_ptr<State> current_state_ptr;

    /**
     * @brief The current operation being executed, essentially the list of states in #state_execution_queue.
     */
    std::string current_operation;

    /**
     * @brief The list of states which shall be executed.
     */
    std::list<std::shared_ptr<State>> state_execution_queue;

    /**
     * @brief Flag for checking if one of the service handlers were called and that the FSM should transition to another
     *        state.
     */
    bool got_new_operation = false;

    /**
     * @brief Used to initialize the service servers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The servers which advertise the operations. 
     */
    ros::ServiceServer take_off_server, travel_server, explore_server, extract_module_server, land_server;

    /**
     * @brief Used to give completion of operations.
     */
    ros::ServiceClient operation_completion_client;

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
     * @brief Service handler for the extract module service.
     * 
     * @param request The extract request.
     * @param response The extract response. 
     * 
     * @return true When the service call has been handled.
     */
    bool extractModule(fluid::ExtractModule::Request& request, fluid::ExtractModule::Response& response);

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
     * @brief Will check if the operation to @p target_state_identifier is valid and update the #state_execution_queue
     *        and #current_operation if it is.
     * 
     * @param target_state_identifier The target state. 
     * @param state_execution_queue The state execution queue for the operation. 
     * 
     * @return Response based on the result of the attempt. 
     */
    Response attemptToCreateOperation(const StateIdentifier& target_state_identifier,
                                      const std::list<std::shared_ptr<State>>& state_execution_queue);
    /**
     * @brief Retrieves the state identifier from @p state_ptr
     * 
     * @param state_ptr The state. 
     * 
     * @return state identifier if state_ptr is not nullptr, #StateIdentifier::UNDEFINED if else.
     */
    StateIdentifier getStateIdentifierForState(std::shared_ptr<State> state_ptr);

    /**
     * @brief Validates if a given operation to @p target_state_identifier is valid from @p current_state_identifier. 
     * 
     * @param current_state_identifier The current state identifier.
     * @param target_state_identifier The target state identifier of the operation.
     * 
     * @return true if the operation is valid.
     */
    bool isValidOperation(const StateIdentifier& current_state_identifier,
                          const StateIdentifier& target_state_identifier) const;

    /**
     * @brief Performs state transition between @p current_state_ptr and @p target_state_ptr.
     * 
     * @param current_state_ptr The current state. 
     * @param target_state_ptr The target state. 
     * 
     * @return The new state (@p target_state_ptr).
     */
    std::shared_ptr<State> performStateTransition(std::shared_ptr<State> current_state_ptr,
                                                  std::shared_ptr<State> target_state_ptr);

   public:
    /**
     * @brief Initializes the service servers.
     */
    OperationHandler();

    /**
     * @brief Starts the main loop.
     */
    void run();
};

#endif
