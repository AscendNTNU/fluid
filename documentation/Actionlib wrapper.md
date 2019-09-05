# Actionlib wrapper

Fluid FSM needs a way to receive an operation request, perform a task which take an arbitrary amount of time, return 
periodic feedback on the progress and respond when the operation has finished. This is where ROS actionlib comes into
the picture. It's important to keep in mind that an operation could be canceled at any time, and therefore the operation
will provide an result with the associated data for when it was canceled.

Be aware that an operation within fluid equals an action in ROS actionlib.



## Operations

All the operation related messages discussed below are located in *ascend_msgs*. Look [here](https://github.com/AscendNTNU/ascend_msgs/blob/master/action/Fluid.action)

### Operation goal
The request a client makes when it wants the FSM to carry out an operation to a given state. It includes:
- type                         (std_msgs/String)
- setpoint                     (geometry_msgs/Point)


### Operation feedback
The action server will provide feedback during the operation. It will include:
- pose_stamped                (geometry_msgs/PoseStamped)

### Operation result
The action server will provide a result when the operation is complete, this is fired once. It includes:
- pose_stamped (geometry_msgs/PoseStamped)

You can check the state of the result in the callback, the different states are documented here [here](http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1SimpleClientGoalState.html#a91066f14351d31404a2179da02c518a0acc5ac2bf0cf2a77d87668ad4be866802).



## General flow
An **operation** within Fluid consist of a series of *states*. So if we issue an operation to move, the series of states will be different from
which state we're currently at. That's why we pass in `type`, which is the state we want to transition in the end. If we for
example are idling at ground, we would have to go through take off, rotate to the move direction and then move for a move operation. 
