
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
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
    private:

                                                                                
        ros::Subscriber pose_subscriber;                                        
        geometry_msgs::PoseStamped current_pose;                                
        void poseCallback(const geometry_msgs::PoseStampedConstPtr pose);

        ros::Subscriber twist_subscriber;                                      
        geometry_msgs::TwistStamped current_twist;                              
        void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);
                
        ros::Publisher setpoint_publisher;                                     


        const bool steady;                                                      ///< Determines whether this state is
                                                                                ///< a state we can be at for longer 
                                                                                ///< periods of time. E.g. hold or idle.

        const bool should_check_obstacle_avoidance_completion;                  ///< Whether it makes sense to check
                                                                                ///< that obstacle avoidance is
                                                                                ///< complete for this state. For e.g. 
                                                                                ///< an init state it wouldn't make 
                                                                                ///< sense.

    protected:

        ros::NodeHandle node_handle;                                            
        mavros_msgs::PositionTarget setpoint;                   


    public:

		const std::string identifier;                                          
  
        const std::string px4_mode;                                            ///< The mode this state represents 
                                                                               ///< within PX4. For example move state
                                                                               ///< would be OFFBOARD while land would 
                                                                               ///< be AUTO_LAND. 


        std::vector<geometry_msgs::Point> path;                                ///< The position targets of the state.

        State(const std::string identifier, 
              const std::string px4_mode, 
              const bool steady, 
              const bool should_check_obstacle_avoidance_completion);

        geometry_msgs::PoseStamped getCurrentPose() const;
        geometry_msgs::TwistStamped getCurrentTwist() const;

        /**
         * Performs the Ros loop for executing logic within this state given the refresh rate.
         *
         * @param tick Called each tick, makes it possible to abort states in the midst of an execution.
         * @param should_halt_if_steady     Will halt at this state if it's steady, is useful
         *                                  if we want to keep at a certain state for some time, e.g. idle
         *                                  or hold.
         */
        virtual void perform(std::function<bool (void)> should_tick, bool should_halt_if_steady);

        void publishSetpoint();
        virtual bool hasFinishedExecution() const = 0;
        virtual void initialize() {}
        virtual void tick() {}

        /**
         * The transition class has to be able to e.g. set the current pose if we transition to a state which requires 
         * to initially know where we are, e. g. land or take off. In that case we can execute the state from the 
         * current pose, and we don't have to wait for the pose callback and thus halt the system.
         */
        friend class Transition;
    };
}
#endif
