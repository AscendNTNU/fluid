#!/usr/bin/env python2

import rospy
import math

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fluid action, including the
# goal message and the result message.

import ascend_msgs.msg
from geometry_msgs.msg import Point


def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    a = 1
    # print("Feedback - " + "Current state: " + feedback.state + "\n Current pose: " + str(feedback.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

def done_callback(state, result):
    b = 1
    # print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state + "\n Final pose: " + str(result.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

path_density = 1

def createPath(x_0, y_0, z_0, x_1, y_1, z_1):
    distance = math.sqrt((x_1-x_0)**2 + (y_1-y_0)**2+ (z_1-z_0)**2)
    new_path = []
    start = Point()
    start.x = x_0
    start.y = y_0
    start.z = z_0
    end = Point()
    end.x = x_1
    end.y = y_1
    end.z = z_1 
    delta_x = end.x - start.x
    delta_y = end.y - start.y
    delta_z = end.z - start.z
    for i in range(0, int(path_density * distance)+1):
        temp = Point()
        temp.x = start.x + i * 1.0 * delta_x / (path_density * distance)
        temp.y = start.y + i * 1.0 * delta_y / (path_density * distance)
        temp.z = start.z + i * 1.0 * delta_z / (path_density * distance)
        new_path.append(temp)
    return new_path

if __name__ == '__main__':
    try:
        rospy.init_node('fluid_client')

        client = actionlib.SimpleActionClient('fluid_operation', ascend_msgs.msg.FluidAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Got contact with server")
        
        # Creates a goal to send to the action server.
        goal = ascend_msgs.msg.FluidGoal()
        """        
	    # The type of operation we want to execute. Can for example be:
	    # - take_off
	    # - land 
	    # - move
	    # - position_follow
        goal.state = "take_off"

        print("Sending goal")
        # Sends the goal to the action server.
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        """
        # Send a new goal

        first = Point()
        first.y = 13 
        first.z = 3

        second = Point()
        second.y = 0
        second.z = 3

        goal.path = [first, second]
        goal.action = "explore"
        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()
        
        """
        first = Point()
        second = Point()
        goal.action = "explore"
        third = Point()
        fourth = Point()

        first.x = 8.0
        first.y = 0.0
        first.z = 2.0

        second.x = 8.0
        second.y = 15.0
        second.z = 2.0

        third.x = -8.0
        third.y = 15.0
        third.z = 2.0

        fourth.x = -8.0
        fourth.y = 0.0
        fourth.z = 2.0

        goal.path = [first, second, third, fourth]

        client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
        client.wait_for_result()
        """
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
