#!/usr/bin/env python2

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

import ascend_msgs.msg
from mavros_msgs.msg import PositionTarget

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('drone_1/fluid_operation', ascend_msgs.msg.FluidAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    setpoint = PositionTarget()
    setpoint.position.x = 0;


    goal = ascend_msgs.msg.FluidGoal()
    goal.type.data = "take_off"
    goal.setpoint = setpoint

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    # return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        fibonacci_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
        print("hello")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
