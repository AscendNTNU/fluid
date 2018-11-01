#include <utility>

#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

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
    protected:

        const unsigned int refresh_rate_ = 20;                         ///< Refresh rate for ros loop.

    public:

        const std::string identifier;                                  ///< Identifier of the state

        std::shared_ptr<fluid::PosePublisher> pose_publisher_p;        ///< Publishes poses.

        geometry_msgs::PoseStamped pose;                               ///< The current pose of the state.
        
        /**
         * Initializes state with an identifier and a refresh rate.
         *
         * @param identifier The identifier of the state.
         * @param refresh_rate Refresh rate of the logic within the state.
         */
        State(  std::string identifier,
                std::shared_ptr<fluid::PosePublisher> pose_publisher_p,
                unsigned int refresh_rate) : identifier(std::move(identifier)), refresh_rate_(refresh_rate) {
            this->pose_publisher_p = std::move(pose_publisher_p);
        }

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
