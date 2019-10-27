#!/usr/bin/env python2

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fluid action, including the
# goal message and the result message.

import ascend_msgs.msg
from geometry_msgs.msg import Point
from ascend_msgs.srv import SplineService, LQR


def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    print("Feedback - " + "Current state: " + feedback.state + "\n Current pose: " + str(feedback.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

def done_callback(state, result):
    print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state + "\n Final pose: " + str(result.pose_stamped))
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
        goal.state = "take_off"
        goal.controller = "racing"

        print("Sending goal")
        # Sends the goal to the action server.
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Send a new goal

        first = Point()
        second = Point()

        first.x = 20.0
        first.y = 20.0
        first.z = 1.0

        second.x = 40.0
        second.y = 0.0
        second.z = 1.0

        goal.path = [first, second]
        goal.state = "move"
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()

        second.z = 15
        goal.path = [second]

        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
