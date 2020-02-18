#!/usr/bin/env python2

import rospy
import math

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fluid action, including the
# goal message and the result message.

import ascend_msgs.msg
from ascend_msgs.msg import PositionYawTarget
from geometry_msgs.msg import Point


def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    # print("Feedback - " + "Current state: " + feedback.state + "\n Current pose: " + str(feedback.pose_stamped))
    # Do something with the pose: feedback.pose_stamped
    return

def done_callback(state, result):
    print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state + "\n Final pose: " + str(result.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

if __name__ == '__main__':
    try:
        rospy.init_node('fluid_client')

        client = actionlib.SimpleActionClient('fluid_operation', ascend_msgs.msg.FluidAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Got contact with server")
        
        # Creates a goal to send to the action server.
        goal = ascend_msgs.msg.FluidGoal()
	#goal.action = "takeoff"
	
	#client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
	#client.wait_for_result()

        first = PositionYawTarget()
        first.point.z = 10

	second = PositionYawTarget()
	second.point.y = 5
	second.point.z = 5

        goal.path = [first,second]
         
	    # The type of operation we want to execute. Can for example be:
	    # - take_off
	    # - land 
	    # - travel
	    # - explore
        goal.action = "explore"

        print("Sending goal")
        # Sends the goal to the action server.
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
   
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
