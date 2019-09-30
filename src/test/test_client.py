#!/usr/bin/env python2

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fluid action, including the
# goal message and the result message.

import ascend_msgs.msg
from geometry_msgs.msg import Point


def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    print("Feedback - " + "Current state: " + feedback.state.data + "\n Current pose: " + str(feedback.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

def done_callback(state, result):
    print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state.data + "\n Final pose: " + str(result.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

if __name__ == '__main__':
    try:
        rospy.init_node('fluid_client')
        
        client = actionlib.SimpleActionClient('fluid_operation', ascend_msgs.msg.FluidAction)
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = ascend_msgs.msg.FluidGoal()
        
	    # The type of operation we want to execute. Can for example be:
	    # - take_off
	    # - land 
	    # - move
	    # - position_follow
        goal.mode.data = "take_off"

        print("Sending goal")
        # Sends the goal to the action server.
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Send a new goal
        goal.setpoint.y = 45.0
        goal.setpoint.x = 0.0
        goal.setpoint.z = 1.2
        goal.mode.data = "move"
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()

        goal.setpoint.z = 20

        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
