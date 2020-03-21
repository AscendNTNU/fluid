/**
 * @file state.h
 */

#ifndef STATE_H
#define STATE_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "state_identifier.h"
#include "type_mask.h"

/** 
 * @brief Interface for states within the finite state machine.
 */
class State {
   private:
    /**
     * @brief Gets the current pose.
     */
    ros::Subscriber pose_subscriber;

    /**
     * @brief Current pose.
     */
    geometry_msgs::PoseStamped current_pose;

    /**
     * @brief Callback for current pose.
     * 
     * @param pose Pose retrieved from the callback. 
     */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr pose);

    /**
     * @brief Gets the current twist.
     */
    ros::Subscriber twist_subscriber;

    /**
     * @brief Current twist.
     */
    geometry_msgs::TwistStamped current_twist;

    /**
     * @brief Callback for current twist.
     * 
     * @param twist Twist retrieved from the callback. 
     */
    void twistCallback(const geometry_msgs::TwistStampedConstPtr twist);

    /**
     * @brief Publishes setpoints.
     * 
     */
    ros::Publisher setpoint_publisher;

    /**
     * @brief Determines whether this state is a state we can be at for longer periods of time. E.g. hold or land. 
     */
    const bool steady;

   protected:
    /**
     * @brief Used to construct the subscribers.
     */
    ros::NodeHandle node_handle;

    /**
     * @brief The setpoint.
     */
    mavros_msgs::PositionTarget setpoint;

    /**
     * @brief Publishes the setpoint.
     */
    void publishSetpoint();

    /**
     * @return true if the state has finished its necessary tasks.
     */
    virtual bool hasFinishedExecution() const = 0;

    /**
     * @brief Initializes the state.
     */
    virtual void initialize() {}

    /**
     * @brief Updates the state logic.
     */
    virtual void tick() {}

    /**
     * @brief Called when the state #hasFinishedExecution.
     */
    virtual void finalize() {}

    /**
     * @return The current pose.
     */
    geometry_msgs::PoseStamped getCurrentPose() const;

    /**
     * @return The current twist.
     */
    geometry_msgs::TwistStamped getCurrentTwist() const;

    /**
     * @return The current yaw.
     */
    float getCurrentYaw() const;

   public:
    /**
     * @brief The identifier for this state.
     */
    const StateIdentifier identifier;

    /**
     * @brief Constructs a new state.
     * 
     * @param identifier The identifier of the state. 
     * @param steady Whether the state is steady, it can be executed for longer periods of time without consequences. 
     */
    State(const StateIdentifier& identifier, const bool& steady);

    /**
     * @brief Performs the Ros loop for executing logic within this state given the refresh rate.
     *
     * @param should_tick               Called each tick, makes it possible to abort states in the midst of an execution.
     * @param should_halt_if_steady     Will halt at this state if it's steady, is useful
     *                                  if we want to keep at a certain state for some time, e.g. idle
     *                                  or hold.
     */
    virtual void perform(std::function<bool(void)> should_tick, bool should_halt_if_steady);

    /**
     * The #OperationHandler class has to be able to e.g. set the current pose if we transition to a state which requires 
     * to initially know where we are, e. g. land or take off. In that case we can execute the state from the 
     * current pose, and we don't have to wait for the pose callback and thus halt the system.
     */
    friend class OperationHandler;
};
#endif
