
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <mavros/mavros_pose_publisher.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include "identifiable.h"

namespace fluid {
    
    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action. It also handles pose publishing.
     */
    class State: public Identifiable {
    protected:

        ros::NodeHandle node_handle_;                                           ///< Node handle for the mavros 
                                                                                ///< pose publisher

        ros::Subscriber pose_subscriber_;                                       ///< Keeps track of the drone's pose 
                                                                                ///< during this state.

        ros::Subscriber twist_subscriber_;                                      ///< Keeps track of the drone's twist 
                                                                                ///< during this state.

        geometry_msgs::PoseStamped current_pose_;                               ///< The current pose of the
                                                                                ///< drone during the state.        


        geometry_msgs::TwistStamped current_twist_;                             ///< The current twist of the  
                                                                                ///< drone during the state.

        ros::Subscriber obstacle_avoidance_completion_subscriber_;              ///< Tells us when the obstacle 
                                                                                ///< avoidance system wants to 
                                                                                ///< go over to position hold

        const bool should_check_obstacle_avoidance_completion_;                 ///< Whether it makes sense to check
                                                                                ///< that obstacle avoidance is
                                                                                ///< complete for this state

        bool obstacle_avoidance_completed_ = false;                             ///< Keeps track of the state with
                                                                                ///< obstacle avoidance.

        /**
         * Gets fired when the state estimation publishes a pose on the given topic.
         */
        void poseCallback(const geometry_msgs::PoseStampedConstPtr pose);

        /**
         * @brief      Retrieves the current twist of the drone.
         *
         * @param[in]  twist  The twist retrieved from the IMU,
         */
        void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);

        /**
         * @brief      Retrieves whether the obstacle avoidance completed or not.
         */
        void obstacleAvoidanceCompletionCallback(const std_msgs::Bool::ConstPtr& completed);

    public:

        const std::string px4_mode;                                            ///< The mode this state represents 
                                                                               ///< within PX4. For example move state
                                                                               ///< would be OFFBOARD. 

        std::shared_ptr<fluid::PosePublisher> position_target_publisher_p;     ///< Publishes poses.

        mavros_msgs::PositionTarget position_target;                           ///< The position target of the state.
        
        /**
         * Sets up the state and the respective publisher and subscriber.
         *
         * @param identifier The identifier of the state.
         * @param px4Mode The mode/state within px4 this state represents.
         * @param pose_subscription_topic The topic to retrieve poses from. 
         * @param twist_subscription_topic The topic to retrieve twists from. 
         * @param position_target_publisher_p Position targets publisher.
         */
        State(std::string identifier,
              std::string px4_mode,
              std::string pose_subscription_topic,
              std::string twist_subscription_topic,
              std::shared_ptr<fluid::PosePublisher> position_target_publisher_p,
              bool should_check_obstacle_avoidance_completion_);

        /**
         * @brief      Returns the current pose, the last pose that the state estimation published on
         *             the given topic.
         */
        geometry_msgs::PoseStamped getCurrentPose();

        /**
         * @brief      Returns the current twist, which is the last twist from the IMU.
         * 
         */
        geometry_msgs::TwistStamped getCurrentTwist();

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
