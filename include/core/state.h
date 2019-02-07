
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <mavros/mavros_pose_publisher.h>
#include "identifiable.h"

namespace fluid {

    typedef std::string StateIdentifier;

    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action. It also handles pose publishing.
     */
    class State: public Identifiable {
    protected:

        ros::NodeHandle node_handle_;                                           ///< Node handle for the mavros 
                                                                                ///< pose publisher


        ros::Subscriber pose_subscriber_;                                       ///< Retrieves poses from mavros

        geometry_msgs::PoseStamped current_pose_;                               ///< Keeps track of the drone's pose 
                                                                                ///< during this state.


        /**
         * Gets fired when the state estimation publishes a pose on the given topic.
         */
        void poseCallback(geometry_msgs::PoseStampedConstPtr pose);

    public:

        ros::Subscriber pose_subscriber;                                       ///< The current pose during the state.

        std::shared_ptr<fluid::PosePublisher> position_target_publisher_p;     ///< Publishes poses.

        mavros_msgs::PositionTarget position_target;                           ///< The position target of the state.
        
        /**
         * Sets up the state and the respective publisher and subscriber.
         *
         * @param identifier The identifier of the state.
         * @param pose_subscription_topic The topic to retrieve poses from. 
         * @param position_target_publisher_p Position targets publisher.
         */
        State(fluid::StateIdentifier identifier,
              std::string pose_subscription_topic,
              std::shared_ptr<fluid::PosePublisher> position_target_publisher_p);

        /**
         * @brief      Returns the current pose, the last pose that the state estimation published on
         *             the given topic.
         */
        geometry_msgs::PoseStamped getCurrentPose();

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
