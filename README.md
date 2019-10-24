# Fluid FSM


## Installation 

### What you need

1. [ROS Melodic](http://wiki.ros.org/melodic/Installation) 
2. [MAVROS](https://dev.px4.io/en/ros/mavros_installation.html)
3. [AirSim (or plain Gazebo with PX4)](https://microsoft.github.io/AirSim/)

## Architecture

Fluid FSM is built around a client-server architecture with ROS actionlib, where a client requests the server to do something. In other words, we (the client) ask the drone (the server) to do **operations** which consist of a series of *states*. 

It also relies on a graph data structure where all the states in the finite state machine lives. This makes it possible to find the path from any
state to another by traversing through the graph. In that way, if we want to move to a certain point, the FSM knows that it has to initialize,
take off, rotate in the movement direction before it moves. This makes the FSM very flexible and reduces complexity on the client side. 

### Different states

The different states are:
- init
- take_off
- hold (hover)
- move 
- rotate
- land
- position_follow (follow a given position, can be a person or an object)
    - In the position follow state the the drone will follow a position given on a topic. If there isn't anything
      published on that topic it will just hover.

The drone will do certain states relative to the current position. E.g. init, take off, land and rotate. So if you request 
the drone to move to a certain position from ground, it will take off from the current position rotate so it's facing the
direction of the movement and then move. 

### Flow

The flow consist of: 

1. The client issuing that we want to go to a certain state
2. The server receiving the request and setting up an operation to that state 
3. The operation figuring out how to get to that state from the current state (a list of states)
4. The operation traversing through each state in that list and executing the respective state logic and transitioning to the next state in the list for each iteration
5. The operation transitioning to a *steady* state after we've finished executing each state. The steady state is a state which, as its name implies,
   can be executed indefintely, e.g. hold (hover) or idle (idle at ground). This makes the FSM ready for a new operation.
6. The operation telling the server that it's done.
7. The server receiving that message and telling the client that the operation has finished.

If the operation failed somewhere along the path (if for example another operation was issued), the server will notify the client that the operation
failed.


## Use

### Run instructions for simulator

1. Make sure you have MAVROS installed and PX4 and gazebo built. 
2. Clone fluid into your catkin workspace in the src-folder.
3. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
4. Start Airsim. 
5. Start fluid server: `roslaunch fluid server_simulator.launch`.
6. Start your client. 

There are some examples in the `src/test` folder.


### Run instructions for physical drone with Pixhawk flight controller

1. Clone fluid into the catkin workspace on the drone. 
2. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
3. Start fluid server via the roslaunch file: `roslaunch fluid_fsm server_pixhawk.launch`.
4. Launch your client node.

## Writing clients

You have to use ROS actionlib in order to communicate with the state machine. Have a look at the python and C++ examples in the [src/test](https://github.com/AscendNTNU/fluid/tree/master/src/test) folder.
