#!/usr/bin/env python2

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

import ascend_msgs.msg
from geometry_msgs.msg import Point

def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    print("Feedback - " + "Current state: " + feedback.state.data)
    
    # Do something with the pose: feedback.pose_stamped

def done_callback(state, result):
    print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state.data)
    # Do something with the pose: feedback.pose_stamped

def send_goal():
    client = actionlib.SimpleActionClient('drone_1/fluid_operation', ascend_msgs.msg.FluidAction)
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ascend_msgs.msg.FluidGoal()
    goal.type.data = "move"
    setpoint = Point()
    setpoint.x = 4.0 
    goal.setpoint = setpoint 
    print("Sending goal")

    # Sends the goal to the action server.
    client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fluid_client')
        print(send_goal())
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
