
#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ascend_msgs/Spline.h>
#include <ascend_msgs/PathOptimizerService.h>
#include <sensor_msgs/Imu.h>

#include "state_identifier.h"
#include "type_mask.h"
#include "controller.h"
#include "trajectory.h"
#include "visualizer.h"
#include "controller.h"

namespace fluid {

    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action. It also handles setpoint publishing.
     */
    class State {
    private:

        ros::NodeHandle node_handle;                                            ///< Node handle for the mavros 
                                                                                ///< pose publisher


        ros::Subscriber pose_subscriber;                                        ///< Keeps track of the drone's pose 
                                                                                ///< during this state.

        geometry_msgs::PoseStamped current_pose;                                ///< The current pose of the
                                                                                ///< drone during the state.        



        void poseCallback(const geometry_msgs::PoseStampedConstPtr pose);



        ros::Subscriber twist_subscriber;                                       ///< Keeps track of the drone's twist 
                                                                                ///< during this state.

        geometry_msgs::TwistStamped current_twist;                              ///< The current twist of the  
                                                                                ///< drone during the state.

        void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);
                
        ros::Subscriber imu_subscriber;                                         

        sensor_msgs::Imu current_imu;

        void imuCallback(const sensor_msgs::ImuConstPtr imu);



        ros::Publisher setpoint_publisher;                                      ///< Publishes setpoints for this state.


        const bool steady;                                                      ///< Determines whether this state is
                                                                                ///< a state we can be at for longer 
                                                                                ///< periods of time. E.g. hold or idle.

        const bool should_check_obstacle_avoidance_completion;                  ///< Whether it makes sense to check
                                                                                ///< that obstacle avoidance is
                                                                                ///< complete for this state. For e.g. 
                                                                                ///< an init state it wouldn't make 
                                                                                ///< sense.

        const bool is_relative;

        Visualizer visualizer;


    protected:

        mavros_msgs::PositionTarget setpoint;                   


    public:

		const std::string identifier;                                          ///< Makes it easy to distinguish 
                                                                               ///< between states 
  
        const std::string px4_mode;                                            ///< The mode this state represents 
                                                                               ///< within PX4. For example move state
                                                                               ///< would be OFFBOARD while land would 
                                                                               ///< be AUTO_LAND. 


        std::vector<geometry_msgs::Point> path;                                ///< The position targets of the state.

        State(const std::string identifier, 
              const std::string px4_mode, 
              const bool steady, 
              const bool should_check_obstacle_avoidance_completion, 
              const bool is_relative);

       /**
         * @brief      Returns the current pose, the last pose that the state estimation published on
         *             the given topic.
         */
        geometry_msgs::PoseStamped getCurrentPose() const;

        /**
         * @brief      Returns the current twist, which is the last twist from the IMU.
         * 
         */
        geometry_msgs::TwistStamped getCurrentTwist() const;

        /**
         * Performs the Ros loop for executing logic within this state given the refresh rate.
         *
         * @param tick Called each tick, makes it possible to abort states in the midst of an execution.
         * @param should_halt_if_steady     Will halt at this state if it's steady, is useful
         *                                  if we want to keep at a certain state for some time, e.g. idle
         *                                  or hold.
         */
        virtual void perform(std::function<bool (void)> tick, bool should_halt_if_steady);

        /**
         * Will publish the current setpoint with the custom obstacle avoidance setpoint message.
         */
        void publishSetpoint();

        /**
         * @return A flag determining whether the state has finished execution.
         */
        virtual bool hasFinishedExecution() const = 0;

        /**
         * Gives the state a chance to do some initial setup before this state is executed.
         */
        virtual void initialize() {}

        /**
         * The transition class has to be able to e.g. set the current pose if we transition to a state which requires 
         * to initially know where we are, e. g. land or take off. In that case we can execute the state from the 
         * current pose, and we don't have to wait for the pose callback and thus halt the system.
         */
        friend class Transition;
    };
}
#endif
