# Actionlib wrapper

Fluid FSM needs a way to receive an operation request, perform a task which take an arbitrary amount of time, return 
periodic feedback on the progress and respond when the operation has finished. This is where ROS actionlib comes into
the picture. It's important to keep in mind that an operation could be canceled at any time, and therefore the operation
will provide an result with the associated data for when it was canceled.

Be aware that an operation within fluid equals an action in ROS actionlib.

As of version 1.0 and 2.0 of Fluid periodic feedback and the final pose of the operation is not implemented as it hasn't been any use of it yet. 
This is because the clients will often just subscribe to the current pose from mavros anyway. This will be looked into in future version if there
is need for it.



## Operations

### Operation goal
The request a client makes when it wants the FSM to carry out an operation to a given state. It includes:
- destination_state_identifier (std_msgs/String)
- setpoint                     (mavros_msgs/PositionTarget)


### Operation feedback
The action server will provide feedback during the operation. It will include:
- current_pose                (geometry_msgs/Pose)

### Operation result
The action server will provide a result when the operation is complete, this is fired once. It includes:
- final_pose                  (geometry_msgs/Pose)
- A flag determining whether the operation completed or not (provided internally by actionlib).



## Action specification
In order for the action client and the action server to communicate we need to define a specification - a few messages
on which they communicate. It is defined in Operation.action in the action folder:

```
# The action goal
std_msgs/String destination_state_identifier
mavros_msgs/PositionTarget setpoint

---
# Action result
geometry_msgs/Pose final_pose

---
# Action feedback
geometry_msgs/Pose current_pose

```



## General flow
An **operation** within Fluid consist of a series of *states*. So if we issue an operation to move, the series of states will be different from
which state we're currently at. That's why we pass in `destination_state_identifier`, which is the state we want to transition in the end. If we for
example are idling at ground, we would have to go through take off, rotate to the move direction and then move for a move operation. 
