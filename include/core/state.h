
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <utility>
#include <memory>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <mavros/mavros_pose_publisher.h>

namespace fluid {

    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action, callbacks when the state started and
     *  finished as well as which states the state can transition to. It also handles pose publishing.
     */
    class State {
    protected:

        const unsigned int refresh_rate_ = 20;                                 ///< Refresh rate for ros loop.

    public:

        const std::string identifier;                                          ///< Identifier of the state

        std::shared_ptr<fluid::PosePublisher> position_target_publisher_p;     ///< Publishes poses.

        mavros_msgs::PositionTarget position_target;                           ///< The position target of the state.
        
        /**
         * Initializes state with an identifier and a refresh rate.
         *
         * @param identifier The identifier of the state.
         * @param position_target_publisher_p Position targets publisher.
         * @param refresh_rate Refresh rate of the logic within the state.
         */
        State(  std::string identifier,
                std::shared_ptr<fluid::PosePublisher> position_target_publisher_p,
                unsigned int refresh_rate) : identifier(identifier), refresh_rate_(refresh_rate) {
            this->position_target_publisher_p = std::move(position_target_publisher_p);
        }

        /**
         * Performs the Ros loop for executing logic within this state given the refresh rate.
         *
         * @param shouldAbort Called each tick, makes it possible to abort states in the midst of an execution.
         */
        virtual void perform(std::function<bool (void)> shouldAbort);

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
