
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ascend_msgs/ObstacleAvoidanceCompletion.h>
#include <mavros_msgs/PositionTarget.h>

#include "state_identifier.h"
#include "type_mask.h"

namespace fluid {
    
    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action. It also handles setpoint publishing.
     */
    class State {
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

        const bool steady_;                                                     ///< Determines whether this state is
                                                                                ///< a state we can be at for longer periods 
                                                                                ///< of time. E.g. hold or idle.

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
        void obstacleAvoidanceCompletionCallback(const ascend_msgs::ObstacleAvoidanceCompletion& msg);

    public:

		const std::string identifier;                                          ///< Makes it easy to distinguish between states 
  
        const std::string px4_mode;                                            ///< The mode this state represents 
                                                                               ///< within PX4. For example move state
                                                                               ///< would be OFFBOARD. 

        const ros::Publisher setpoint_publisher;                               ///< Publishes setpoints for this state.

        mavros_msgs::PositionTarget setpoint;                                  ///< The position target of the state.
        
        /**
         * Sets up the state and the respective publishers and subscribers.
         *
         * @param identifier The identifier of the state.
         * @param px4_mode The mode/state within px4 this state represents.
         * @param steady Defines if this state is a state we can be at for longer periods of time. E.g. idle
         *               or hold.
         * @param should_check_obstacle_avoidance_completion Determines whether the state should be affected
         *                                                   by obstacle avoidance saying we've reached a setpoint or not.
         *                                                   Used in states which require to run through a set of instructions,
         *                                                   e.g. an init state.
         */
        State(std::string identifier,
              std::string px4_mode,
              bool steady,
              bool should_check_obstacle_avoidance_completion);


        /**
         * @breif      A way to set the pose of the state. 
         * 
         *             This is useful if we want to 
         *             transition to a state which requires to initially know where we are, e. g. 
         *             land or take off. In that case we can execute the state from the current 
         *             pose, and we don't have to wait for the pose callback and thus halt the system.
         */
        void setCurrentPose(geometry_msgs::PoseStamped currentPose);

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
         * @param should_halt_if_steady     Will halt at this state if it's steady, is useful
         *                                  if we want to keep at a certain state for some time, e.g. idle
         *                                  or hold.
         */
        virtual void perform(std::function<bool (void)> shouldAbort, bool should_halt_if_steady);

        /**
         * @return A flag determining whether the state has finished execution.
         */
        virtual bool hasFinishedExecution() = 0;

        /**
         * Gives the state a chance to do some initial setup before this state is executed.
         */
        virtual void initialize() {}

        /**
         * Executes logic at given refresh rate.
         */
        virtual void tick() = 0;
    };
}
#endif
