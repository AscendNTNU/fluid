//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_CLIENT_H
#define FLUID_FSM_CLIENT_H

#include <utility>
#include <string>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <mavros_msgs/PositionTarget.h>

#include <fluid/OperationAction.h>

#include "operation.h"
#include "core.h"

namespace fluid {

/** \class Client
 *  \brief Encapsulates a ROS actionlib client which requests operations.
 */
class Client {

    typedef actionlib::SimpleActionClient<fluid::OperationAction> ActionlibClient;

  private:

    const std::string name_space;                 ///< The namespace of the client, have to correspond to the server's namespace.

    const double timeout_value_;                  ///< The time the the client will wait for a response from
                          				                ///< the server

    /**
     * @brief      Waits for the timeout or the completion from the operation server.
     */
    void waitForResult(std::string identifier, 
                       mavros_msgs::PositionTarget setpoint,
		                   std::function<void(bool)> completion_handler);

  public:

    /** Initializes the client with a timeout value.
     *
     * @param name_sspace The namespace of the client server pair, this has to correspond to the server's namespace.
     * @param timeout_value The time the operation client waits for a response from the operation server.
     */
    Client(std::string name_space, double timeout_value) : name_space(name_space), timeout_value_(timeout_value) {}

    /**
     * Issues a take off request to the given height.
     * 
     * @param height The height to take off to.
     * @param completion_handler Fired when this operation has finished (or timed out) 
     */
    void requestTakeOff(double height, std::function<void (bool)> completion_handler);

    /**
     * Issues a take off request to the default height.
     * 
     * @param completion_handler Fired when this operation has finished (or timed out) 
     */
    void requestTakeOff(std::function<void (bool)> completion_handler);

     /**
     * Issues a move request to a given setpoint.
     * 
     * @param completion_handler Fired when this operation has finished (or timed out) 
     */
    void requestMove(mavros_msgs::PositionTarget setpoint, std::function<void (bool)> completion_handler);

    /**
     * Issues a land request at the given position.
     * 
     * @param completion_handler Fired when this operation has finished (or timed out) 
     */
    void requestLand(std::function<void (bool)> completion_handler);

     /**
     * Issues a position follow request.
     */
    void requestPositionFollow();

    /**
     * Requests an operation with a given setpoint. This function will send a request to a server
     * listening on the fluid operation domain. This function is asynchronous.
     *
     * @param identifier The state to traverse to.
     * @param setpoint The setpoint of the operation.
     * @param completion_handler Gets fired when the operation finished, includes a flag whether the operation finished
     *                           before timeout or not.
     */
    void requestOperationToState(std::string identifier, mavros_msgs::PositionTarget setpoint,
			  std::function<void(bool)> completion_handler);
};
}

#endif
