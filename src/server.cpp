#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <ascend_msgs/FluidGoal.h>

#include <cmath>
#include <assert.h>
#include <algorithm>

#include "server.h"
#include "core.h"
#include "util.h"

fluid::Server::Server() : actionlib_server_(node_handle_, 
                                            "fluid_operation",
                                            false) {
    

    actionlib_server_.registerPreemptCallback(boost::bind(&Server::preemptCallback, this));
    actionlib_server_.start();
}

std::shared_ptr<fluid::Operation> fluid::Server::retrieveNewOperation() {

    // We accept the new goal and initialize variables for target pose and the type of operation identifier.
    // This is necessary in order to modify some of them before we initiate the different operations further down.
    // E.g. the init operation shouldn't be called with a different pose than (0, 0, 0), so we make sure this is the
    // case.

    if (!actionlib_server_.isNewGoalAvailable()) {
        return nullptr;
    }

    if (actionlib_server_.isActive()) {
        actionlib_server_.setPreempted();
    }

    auto goal = actionlib_server_.acceptNewGoal();
    std::vector<geometry_msgs::Point> path = goal->path;
    std::string destination_identifier = goal->state.data;
    std::string controller = goal->controller.data;

    Core::getStatusPublisherPtr()->status.path = path;

    float distanceToEndpoint = Util::distanceBetween(fluid::Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position, path.back());
    bool shouldIncludeMove = distanceToEndpoint >= fluid::Core::distance_completion_threshold;

    auto states = fluid::Core::getGraphPtr()->getPathToEndState(fluid::Core::getGraphPtr()->current_state_ptr->identifier, destination_identifier, shouldIncludeMove);
    ROS_INFO_STREAM("New operation requested to state: " << destination_identifier);

    std::stringstream stringstream;

    for (auto state : states) {
        stringstream << state->identifier << " ";
    }

    ROS_INFO_STREAM("Will traverse through: " << stringstream.str() << "\n");

    return std::make_shared<fluid::Operation>(destination_identifier, path);
}

void fluid::Server::preemptCallback() {
    actionlib_server_.setPreempted();
}

void fluid::Server::start() {

    ros::Rate rate(fluid::Core::refresh_rate);

    // Main loop of Fluid FSM. This is where all the magic happens. If a new operaiton is requested, the 
    // new operation requested flag is set and we set up the requirements for that operation to run. When it
    // it runs we check every tick if a new operation is requested and abort from the current operation if
    // that is the case. 

    std::shared_ptr<fluid::Operation> current_operation_ptr;
    std::shared_ptr<fluid::State> last_state_ptr;

    while (ros::ok()) {

        // Execute the operation if there is any
        if (current_operation_ptr) {

            fluid::Core::getStatusPublisherPtr()->status.current_operation = current_operation_ptr->getDestinationStateIdentifier();

            current_operation_ptr->perform(

                [&]() -> bool {

                    ascend_msgs::FluidFeedback feedback;
                    std::shared_ptr<fluid::State> current_state_ptr = Core::getGraphPtr()->current_state_ptr;
                    feedback.pose_stamped = current_state_ptr->getCurrentPose();
                    feedback.state.data = current_state_ptr->identifier;
                    actionlib_server_.publishFeedback(feedback);

                    return !actionlib_server_.isPreemptRequested() && ros::ok(); 
                },

                [&](bool completed) {
                    // We completed the operation and want to end at the final state of the operation (e.g. hold)
                    // state for move. One can think of this step as making sure that the state machine is at a
                    // state where it's easy to execute a new operation. If we did not complete the operation has
                    // already transitioned to a steady state and we just set the last state to that state.
                    if (completed) {
                        last_state_ptr = current_operation_ptr->getFinalStatePtr();
                    }
                    else {
                        last_state_ptr = fluid::Core::getGraphPtr()->current_state_ptr;
                    }


                    // Will notify the operation client what the outcome of the operation was. This will end up
                    // calling the callback that the operation client set up for completion.
                    
                    if (!actionlib_server_.isActive()) {
                        return;
                    }
                    
                    ascend_msgs::FluidResult result;

                    if (completed) {
                        ROS_INFO_STREAM("Operation completed.");
                        result.pose_stamped = Core::getGraphPtr()->current_state_ptr->getCurrentPose();
                        result.state.data = last_state_ptr->identifier;
                        actionlib_server_.setSucceeded(result);
                    }
                    else {
                        ROS_INFO_STREAM("Operation cancelled.");
                        result.pose_stamped = Core::getGraphPtr()->current_state_ptr->getCurrentPose();
                        result.state.data = last_state_ptr->identifier;
                        actionlib_server_.setPreempted(result);
                    }
                });

            current_operation_ptr.reset();
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {
            fluid::Core::getStatusPublisherPtr()->status.current_operation = "none";

            if (last_state_ptr) {
                last_state_ptr->perform([&]() -> bool {
                    // We abort the execution of the current state if there is a new operation.
                    return !actionlib_server_.isNewGoalAvailable();
                }, true);
            }
        }

        fluid::Core::getStatusPublisherPtr()->publish();

        // Setup for the new operation.
        if (actionlib_server_.isNewGoalAvailable() && !actionlib_server_.isActive()) {
            current_operation_ptr = retrieveNewOperation();
        }

        ros::spinOnce();
        rate.sleep();
    }
}
