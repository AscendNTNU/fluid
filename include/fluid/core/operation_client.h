//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_OPERATION_CLIENT_H
#define FLUID_FSM_OPERATION_CLIENT_H

#include <utility>
#include <string>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <fluid/OperationAction.h>
#include <geometry_msgs/Pose.h>
#include <fluid/core/operation/operation.h>

namespace fluid {

/** \class OperationClient
 *  \brief Encapsulates a ROS action client which requests operations.
 */
class OperationClient {

    typedef actionlib::SimpleActionClient<fluid::OperationAction> Client;

  private:

    const std::string name_space; ///< The namespace of the client, have to correspond to the server's namespace.

    const double timeout_value_; ///< The time the the client will wait for a response from
				                 ///< the server

    /**
     * @brief      Waits for the timeout or the completion from the operation server.
     */
    void waitForResult(std::string operation_identifier, 
                       geometry_msgs::Pose target_pose,
		               std::function<void(bool)> completion_handler);

  public:
    /** Initializes the operation client with a timeout value.
     *
     * @param name_sspace The namespace of the client server pair, this has to correspond to the server's namespace.
     * @param timeout_value The time the operation client waits for a response from the operation server.
     */
    OperationClient(std::string name_space, double timeout_value) : name_space(name_space), timeout_value_(timeout_value) {}

    /**
     * Requests an operation with a given target pose. This function will send a request to a server
     * listening on the fluid operation domain. This function is asynchronous.
     *
     * @param operation_identifier The type of operation to execute.
     * @param target_pose The target pose of the operation.
     * @param completion_handler Gets fired when the operation finished, includes a flag whether the operation finished
     *                           before timeout or not.
     */
    void requestOperation(std::string operation_identifier, geometry_msgs::Pose target_pose,
			  std::function<void(bool)> completion_handler);
};
}

#endif // FLUID_FSM_OPERATION_CLIENT_H
