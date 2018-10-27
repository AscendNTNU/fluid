#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include "../states/state_identifier.h"
#include <memory>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <mavros/mavros_pose_publisher.h>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {

    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action, callbacks when the state started and
     *  finished as well as which states the state can transition to. It also handles pose publishing.
     */
    class State {
    private:
        static ros::NodeHandle node_handle_;                           ///< Handle for the mavros pose publisher.

        const unsigned int refresh_rate_ = 20;                         ///< Refresh rate for ros loop.

    public:

        const fluid::StateIdentifier identifier;                       ///< Identifier of the state

        static fluid::MavrosPosePublisher publisher;                   ///< Publishes poses to Pixhawk through mavros.

        geometry_msgs::PoseStamped pose;                               ///< The current pose of the state.
        
        /**
         * Initializes state with an identifier and a refresh rate.
         *
         * @param identifier The identifier of the state.
         * @param refresh_rate Refresh rate of the logic within the state.
         */
        State(fluid::StateIdentifier identifier, unsigned int refresh_rate): identifier(identifier),
                                                                             refresh_rate_(refresh_rate) {}

        /**
         * Performs the Ros loop for executing logic within this state given the refresh rate.
         */
        void perform();

        /**
         * @return A flag determining whether the state has finished execution.
         */
        virtual bool hasFinishedExecution() = 0;

        /**
         * Executes logic at given refresh rate.
         */
        virtual void tick() = 0;
    };
}
#endif
