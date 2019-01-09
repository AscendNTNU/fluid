//
// Created by simengangstad on 08.01.19.
//

#ifndef FLUID_FSM_NAVIGATOR_STATE_H
#define FLUID_FSM_NAVIGATOR_STATE_H

#include "../core/state.h"
#include "../core/operation/operation.h"
#include "navigator_pose_publisher.h"

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {
    /**
     * \class NavigatorState
     * \brief Encapsulates a state which publishes poses through the navigator interface.
     */
    class NavigatorState: public State {

    private:


        ros::Subscriber pose_subscriber_;                        ///< Retrieves poses from mavros

    protected:

        ros::NodeHandlePtr node_handle_p;                        ///< Used for the pose subscriber and the pose 
                                                                 ///< pose publisher

        geometry_msgs::PoseStamped current_position_;            ///< The current pose during the state.

        /**
         * Gets fired when state estimator publishes poses on the specified topic.
         */
        void poseCallback(geometry_msgs::PoseStampedConstPtr pose);

    public:

        /**
         * Initiializes the navigator state with an identifier.
         *
         * @param node_handle_p Node handle to interact with ROS topics.
         * @param identifier The identifier of the state.
         */
        // TODO: Topic is temporary
        NavigatorState(ros::NodeHandlePtr node_handle_p, fluid::OperationIdentifier identifier) :
        State(std::move(identifier), std::make_shared<fluid::NavigatorPosePublisher>(node_handle_p, "navigator_pose_topic", 1000), 20),
        node_handle_p(node_handle_p),
        pose_subscriber_(node_handle_p->subscribe("state_estimator_pose", 1000, &NavigatorState::poseCallback, this))
        {}
    };
}

#endif //FLUID_FSM_NAVIGATOR_STATE_H
